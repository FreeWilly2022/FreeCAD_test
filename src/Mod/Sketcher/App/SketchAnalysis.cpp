// SPDX-License-Identifier: LGPL-2.1-or-later

/***************************************************************************
 *   Copyright (c) 2018 Abdullah Tahiri <abdullah.tahiri.yo@gmail.com>     *
 *   Copyright (c) 2013 Werner Mayer <wmayer[at]users.sourceforge.net>     *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/

#include <cmath>

#include <BRep_Tool.hxx>
#include <Precision.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Pnt.hxx>

#include <App/Document.h>
#include <Base/Console.h>

#include "GeometryFacade.h"
#include "SketchAnalysis.h"
#include "SketchObject.h"
#include <App/PropertyLinks.h>

#include "Base/Vector3D.h"
#include "ExternalGeometryFacade.h"
#include <Mod/Part/App/Geometry.h>
#include <Mod/Sketcher/App/ExternalGeometryExtension.h>


using namespace Sketcher;

SketchAnalysis::SketchAnalysis(Sketcher::SketchObject* Obj)
    : sketch(Obj)
{}

SketchAnalysis::~SketchAnalysis() = default;

namespace
{
struct VertexIds
{
    Base::Vector3d v;
    int GeoId {};
    Sketcher::PointPos PosId {};
};

struct Vertex_Less
{
    explicit Vertex_Less(double tolerance)
        : tolerance(tolerance)
    {}
    bool operator()(const VertexIds& x, const VertexIds& y) const
    {
        if (fabs(x.v.x - y.v.x) > tolerance) {
            return x.v.x < y.v.x;
        }
        if (fabs(x.v.y - y.v.y) > tolerance) {
            return x.v.y < y.v.y;
        }
        if (fabs(x.v.z - y.v.z) > tolerance) {
            return x.v.z < y.v.z;
        }
        return false;  // points are considered to be equal
    }

private:
    double tolerance;
};

struct VertexID_Less
{
    bool operator()(const VertexIds& x, const VertexIds& y) const
    {
        return (x.GeoId < y.GeoId || ((x.GeoId == y.GeoId) && (x.PosId < y.PosId)));
    }
};

struct Vertex_EqualTo
{
    explicit Vertex_EqualTo(double tolerance)
        : tolerance(tolerance)
    {}
    bool operator()(const VertexIds& x, const VertexIds& y) const
    {
        if (fabs(x.v.x - y.v.x) <= tolerance) {
            if (fabs(x.v.y - y.v.y) <= tolerance) {
                if (fabs(x.v.z - y.v.z) <= tolerance) {
                    return true;
                }
            }
        }
        return false;
    }

private:
    double tolerance;
};

struct EdgeIds
{
    double l {};
    int GeoId {};
};

struct Edge_Less
{
    explicit Edge_Less(double tolerance)
        : tolerance(tolerance)
    {}
    bool operator()(const EdgeIds& x, const EdgeIds& y) const
    {
        if (fabs(x.l - y.l) > tolerance) {
            return x.l < y.l;
        }
        return false;  // points are considered to be equal
    }

private:
    double tolerance;
};

struct Edge_EqualTo
{
    explicit Edge_EqualTo(double tolerance)
        : tolerance(tolerance)
    {}
    bool operator()(const EdgeIds& x, const EdgeIds& y) const
    {
        return (fabs(x.l - y.l) <= tolerance);
    }

private:
    double tolerance;
};

struct PointConstraints
{
    void addGeometry(const Part::Geometry* geo, int index)
    {
        if (const auto* segm = dynamic_cast<const Part::GeomLineSegment*>(geo)) {
            addLineSegment(segm, index);
        }
        else if (const auto* segm = dynamic_cast<const Part::GeomArcOfCircle*>(geo)) {
            addArcOfCircle(segm, index);
        }
        else if (const auto* segm = dynamic_cast<const Part::GeomArcOfEllipse*>(geo)) {
            addArcOfEllipse(segm, index);
        }
        else if (const auto* segm = dynamic_cast<const Part::GeomArcOfHyperbola*>(geo)) {
            addArcOfHyperbola(segm, index);
        }
        else if (const auto* segm = dynamic_cast<const Part::GeomArcOfParabola*>(geo)) {
            addArcOfParabola(segm, index);
        }
        else if (const auto* segm = dynamic_cast<const Part::GeomBSplineCurve*>(geo)) {
            addBSplineCurve(segm, index);
        }
    }

    void addLineSegment(const Part::GeomLineSegment* segm, int index)
    {
        VertexIds id;
        id.GeoId = index;
        id.PosId = Sketcher::PointPos::start;
        id.v = segm->getStartPoint();
        vertexIds.push_back(id);
        id.GeoId = index;
        id.PosId = Sketcher::PointPos::end;
        id.v = segm->getEndPoint();
        vertexIds.push_back(id);
    }

    void addArcOfCircle(const Part::GeomArcOfCircle* segm, int index)
    {
        VertexIds id;
        id.GeoId = index;
        id.PosId = Sketcher::PointPos::start;
        id.v = segm->getStartPoint(/*emulateCCWXY=*/true);
        vertexIds.push_back(id);
        id.GeoId = index;
        id.PosId = Sketcher::PointPos::end;
        id.v = segm->getEndPoint(/*emulateCCWXY=*/true);
        vertexIds.push_back(id);
    }

    void addArcOfEllipse(const Part::GeomArcOfEllipse* segm, int index)
    {
        VertexIds id;
        id.GeoId = index;
        id.PosId = Sketcher::PointPos::start;
        id.v = segm->getStartPoint(/*emulateCCWXY=*/true);
        vertexIds.push_back(id);
        id.GeoId = index;
        id.PosId = Sketcher::PointPos::end;
        id.v = segm->getEndPoint(/*emulateCCWXY=*/true);
        vertexIds.push_back(id);
    }

    void addArcOfHyperbola(const Part::GeomArcOfHyperbola* segm, int index)
    {
        VertexIds id;
        id.GeoId = index;
        id.PosId = Sketcher::PointPos::start;
        id.v = segm->getStartPoint();
        vertexIds.push_back(id);
        id.GeoId = index;
        id.PosId = Sketcher::PointPos::end;
        id.v = segm->getEndPoint();
        vertexIds.push_back(id);
    }

    void addArcOfParabola(const Part::GeomArcOfParabola* segm, int index)
    {
        VertexIds id;
        id.GeoId = index;
        id.PosId = Sketcher::PointPos::start;
        id.v = segm->getStartPoint();
        vertexIds.push_back(id);
        id.GeoId = index;
        id.PosId = Sketcher::PointPos::end;
        id.v = segm->getEndPoint();
        vertexIds.push_back(id);
    }

    void addBSplineCurve(const Part::GeomBSplineCurve* segm, int index)
    {
        VertexIds id;
        id.GeoId = index;
        id.PosId = Sketcher::PointPos::start;
        id.v = segm->getStartPoint();
        vertexIds.push_back(id);
        id.GeoId = index;
        id.PosId = Sketcher::PointPos::end;
        id.v = segm->getEndPoint();
        vertexIds.push_back(id);
    }

    std::list<ConstraintIds> getMissingCoincidences(
        std::vector<Sketcher::Constraint*>& allcoincid,
        double precision
    )
    {
        std::list<ConstraintIds> missingCoincidences;  // Holds the list of missing coincidences

        // Sort points in geographic order
        std::sort(vertexIds.begin(), vertexIds.end(), Vertex_Less(precision));

        auto vt = vertexIds.begin();
        Vertex_EqualTo pred(precision);

        // Comparing existing constraints and find missing ones

        while (vt < vertexIds.end()) {
            // Seeking for adjacent group of vertices
            vt = std::adjacent_find(vt, vertexIds.end(), pred);
            if (vt < vertexIds.end()) {  // If one found
                std::vector<VertexIds>::iterator vn;
                // Holds a single group of adjacent vertices
                std::set<VertexIds, VertexID_Less> vertexGrp;
                // Extract the group of adjacent vertices
                vertexGrp.insert(*vt);
                for (vn = vt + 1; vn < vertexIds.end(); ++vn) {
                    if (pred(*vt, *vn)) {
                        vertexGrp.insert(*vn);
                    }
                    else {
                        break;
                    }
                }

                // Holds groups of coincident vertices
                std::vector<std::set<VertexIds, VertexID_Less>> coincVertexGrps;

                // Decompose the group of adjacent vertices into groups of coincident vertices
                // Going through existent coincidences
                // Search only constraints associated with geometry

                for (auto& coincidence : allcoincid) {
                    VertexIds v1;
                    VertexIds v2;
                    bool v1_found = false, v2_found = false;

                    // Find the actual vertex data (including coordinates) for this constraint
                    for (const auto& vid : vertexIds) {
                        if (vid.GeoId == coincidence->First && vid.PosId == coincidence->FirstPos) {
                            v1 = vid;
                            v1_found = true;
                        }
                        if (vid.GeoId == coincidence->Second && vid.PosId == coincidence->SecondPos) {
                            v2 = vid;
                            v2_found = true;
                        }
                        if (v1_found && v2_found) {
                            break;
                        }
                    }

                    if (!v1_found || !v2_found) {
                        continue;
                    }

                    // If the constraint says they are connected, but they aren't
                    // close, they should NOT be pulled into the same coincident group.
                    double dist = (v1.v - v2.v).Length();

                    if (dist > Precision::Confusion()) {
                        continue;
                    }

                    // Existing extraction logic (now using verified v1 and v2)
                    auto nv1 = vertexGrp.extract(v1);
                    auto nv2 = vertexGrp.extract(v2);

                    // Maybe if both empty, they already have been extracted by other coincidences
                    // We have to check in existing coincident groups and eventually merge
                    if (nv1.empty() && nv2.empty()) {
                        std::set<VertexIds, VertexID_Less>* tempGrp = nullptr;
                        for (auto it = coincVertexGrps.begin(); it < coincVertexGrps.end(); ++it) {
                            if ((it->find(v1) != it->end()) || (it->find(v2) != it->end())) {
                                if (!tempGrp) {
                                    tempGrp = &*it;
                                }
                                else {
                                    tempGrp->insert(it->begin(), it->end());
                                    coincVertexGrps.erase(it);
                                    break;
                                }
                            }
                        }
                        continue;
                    }

                    // Look if one of the constrained vertices is already in a group of coincident
                    // vertices
                    for (std::set<VertexIds, VertexID_Less>& grp : coincVertexGrps) {
                        if ((grp.find(v1) != grp.end()) || (grp.find(v2) != grp.end())) {
                            // If yes add them to the existing group
                            if (!nv1.empty()) {
                                grp.insert(nv1.value());
                            }
                            if (!nv2.empty()) {
                                grp.insert(nv2.value());
                            }
                            continue;
                        }
                    }

                    if (nv1.empty() || nv2.empty()) {
                        continue;
                    }

                    // If no, create a new group of coincident vertices
                    std::set<VertexIds, VertexID_Less> newGrp;
                    newGrp.insert(nv1.value());
                    newGrp.insert(nv2.value());
                    coincVertexGrps.push_back(newGrp);
                }

                // If there are remaining vertices in the adjacent group (not in any existing
                // constraint) add them as being each a separate coincident group
                for (auto& lonept : vertexGrp) {
                    std::set<VertexIds, VertexID_Less> newGrp;
                    newGrp.insert(lonept);
                    coincVertexGrps.push_back(newGrp);
                }

                // If there is more than 1 coincident group into adjacent group, constraint(s)
                // is(are) missing. Virtually generate the missing constraint(s)
                if (coincVertexGrps.size() > 1) {
                    std::vector<std::set<VertexIds, VertexID_Less>>::iterator vn;
                    // Starting from the 2nd coincident group, generate a constraint between
                    // this group first vertex, and previous group first vertex
                    for (vn = coincVertexGrps.begin() + 1; vn < coincVertexGrps.end(); ++vn) {
                        ConstraintIds id;
                        id.Type = Coincident;  // default point on point restriction
                        id.v = (vn - 1)->begin()->v;
                        id.First = (vn - 1)->begin()->GeoId;
                        id.FirstPos = (vn - 1)->begin()->PosId;
                        id.Second = vn->begin()->GeoId;
                        id.SecondPos = vn->begin()->PosId;
                        missingCoincidences.push_back(id);
                    }
                }

                vt = vn;
            }
        }
        return missingCoincidences;
    }

private:
    // Holds a list of all vertices in the sketch
    std::vector<VertexIds> vertexIds;
};

struct EqualityConstraints
{
    void addGeometry(const Part::Geometry* geo, int index)
    {
        if (const auto* segm = dynamic_cast<const Part::GeomLineSegment*>(geo)) {
            addLineSegment(segm, index);
        }
        else if (const auto* segm = dynamic_cast<const Part::GeomArcOfCircle*>(geo)) {
            addArcOfCircle(segm, index);
        }
        else if (const auto* segm = dynamic_cast<const Part::GeomCircle*>(geo)) {
            addCircle(segm, index);
        }
    }

    void addLineSegment(const Part::GeomLineSegment* segm, int index)
    {
        EdgeIds id;
        id.GeoId = index;
        id.l = (segm->getEndPoint() - segm->getStartPoint()).Length();
        lineedgeIds.push_back(id);
    }

    void addArcOfCircle(const Part::GeomArcOfCircle* segm, int index)
    {
        EdgeIds id;
        id.GeoId = index;
        id.l = segm->getRadius();
        radiusedgeIds.push_back(id);
    }

    void addCircle(const Part::GeomCircle* segm, int index)
    {
        EdgeIds id;
        id.GeoId = index;
        id.l = segm->getRadius();
        radiusedgeIds.push_back(id);
    }

    std::list<ConstraintIds> getEqualLines(double precision)
    {
        std::sort(lineedgeIds.begin(), lineedgeIds.end(), Edge_Less(precision));
        auto vt = lineedgeIds.begin();
        Edge_EqualTo pred(precision);

        std::list<ConstraintIds> equallines;
        // Make a list of constraint we expect for coincident vertexes
        while (vt < lineedgeIds.end()) {
            // get first item whose adjacent element has the same vertex coordinates
            vt = std::adjacent_find(vt, lineedgeIds.end(), pred);
            if (vt < lineedgeIds.end()) {
                std::vector<EdgeIds>::iterator vn;
                for (vn = vt + 1; vn != lineedgeIds.end(); ++vn) {
                    if (pred(*vt, *vn)) {
                        ConstraintIds id;
                        id.Type = Equal;
                        id.v.x = vt->l;
                        id.First = vt->GeoId;
                        id.FirstPos = Sketcher::PointPos::none;
                        id.Second = vn->GeoId;
                        id.SecondPos = Sketcher::PointPos::none;
                        equallines.push_back(id);
                    }
                    else {
                        break;
                    }
                }

                vt = vn;
            }
        }

        return equallines;
    }

    std::list<ConstraintIds> getEqualRadius(double precision)
    {
        std::sort(radiusedgeIds.begin(), radiusedgeIds.end(), Edge_Less(precision));
        auto vt = radiusedgeIds.begin();
        Edge_EqualTo pred(precision);

        std::list<ConstraintIds> equalradius;
        // Make a list of constraint we expect for coincident vertexes
        while (vt < radiusedgeIds.end()) {
            // get first item whose adjacent element has the same vertex coordinates
            vt = std::adjacent_find(vt, radiusedgeIds.end(), pred);
            if (vt < radiusedgeIds.end()) {
                std::vector<EdgeIds>::iterator vn;
                for (vn = vt + 1; vn != radiusedgeIds.end(); ++vn) {
                    if (pred(*vt, *vn)) {
                        ConstraintIds id;
                        id.Type = Equal;
                        id.v.x = vt->l;
                        id.First = vt->GeoId;
                        id.FirstPos = Sketcher::PointPos::none;
                        id.Second = vn->GeoId;
                        id.SecondPos = Sketcher::PointPos::none;
                        equalradius.push_back(id);
                    }
                    else {
                        break;
                    }
                }

                vt = vn;
            }
        }

        return equalradius;
    }

private:
    std::vector<EdgeIds> lineedgeIds;
    std::vector<EdgeIds> radiusedgeIds;
};

}  // namespace

int SketchAnalysis::detectMissingPointOnPointConstraints(double precision, bool includeconstruction /*=true*/)
{
    PointConstraints pointConstr;

    // Build the list of sketch vertices
    const std::vector<Part::Geometry*>& geom = sketch->getInternalGeometry();
    for (std::size_t i = 0; i < geom.size(); i++) {

        auto gf = GeometryFacade::getFacade(geom[i]);

        if (gf->getConstruction() && !includeconstruction) {
            continue;
        }

        pointConstr.addGeometry(gf->getGeometry(), int(i));
    }

    // Build a list of all coincidences in the sketch

    // clang-format on

    std::vector<Sketcher::Constraint*> coincidences;
    coincidences.reserve(sketch->Constraints.getValues().size());

    for (auto* constraint : sketch->Constraints.getValues()) {
        if (constraint->Type == Sketcher::Coincident) {
            coincidences.push_back(constraint);
        }

        // a Tangent constraint can act as a Coincident constraint depending on what geometry was
        // selected
        else if (constraint->Type == Sketcher::Tangent) {
            // PointPos::start (1) and PointPos::end (2) are the endpoints
            bool firstIsPoint
                = (constraint->FirstPos == Sketcher::PointPos::start
                   || constraint->FirstPos == Sketcher::PointPos::end);
            bool secondIsPoint
                = (constraint->SecondPos == Sketcher::PointPos::start
                   || constraint->SecondPos == Sketcher::PointPos::end);

            if (firstIsPoint && secondIsPoint) {
                coincidences.push_back(constraint);
            }
        }
        // a Perpendicular constraint can act as a Coincident constraint depending on what geometry
        // was selected
        else if (constraint->Type == Sketcher::Perpendicular) {
            // Check if it's linking specific points or just lines
            if (constraint->FirstPos != Sketcher::PointPos::none
                && constraint->SecondPos != Sketcher::PointPos::none) {
                coincidences.push_back(constraint);
            }
        }
        // a Symmetric constraint can act as a Coincident constraint depending on what geometry was
        // selected
        else if (constraint->Type == Sketcher::Symmetric) {
            // Case 1: Two points symmetric about a reference point/line
            // In FreeCAD, a Symmetric constraint involves 3 elements:
            // First, Second (the symmetric points) and Third (the reference)

            bool isPointToPoint
                = (constraint->FirstPos != Sketcher::PointPos::none
                   && constraint->SecondPos != Sketcher::PointPos::none);

            if (constraint->First == constraint->Second) {
                // This is a mathematical "collapse" where a point is symmetric to itself.
                // It implies the point must lie exactly on the reference (Third).
                coincidences.push_back(constraint);
            }

            // Case 2: Checking if the constraint targets specific vertices (endpoints)
            else if (isPointToPoint) {
                coincidences.push_back(constraint);
            }
        }
    }

    // Holds the list of missing coincidences
    std::list<ConstraintIds> missingCoincidences
        = pointConstr.getMissingCoincidences(coincidences, precision);

    // Update list of missing constraints stored as member variable of sketch
    this->vertexConstraints.clear();
    this->vertexConstraints.reserve(missingCoincidences.size());

    for (auto& coincidence : missingCoincidences) {
        this->vertexConstraints.push_back(coincidence);
    }

    // Return number of missing constraints
    return int(this->vertexConstraints.size());
}

void SketchAnalysis::analyseMissingPointOnPointCoincident(double angleprecision)
{
    for (auto& vc : vertexConstraints) {

        auto geo1 = sketch->getGeometry(vc.First);
        auto geo2 = sketch->getGeometry(vc.Second);

        // tangency point-on-point
        const auto* curve1 = dynamic_cast<const Part::GeomCurve*>(geo1);
        const auto* curve2 = dynamic_cast<const Part::GeomCurve*>(geo2);

        if (curve1 && curve2) {

            const auto* segm1 = dynamic_cast<const Part::GeomLineSegment*>(geo1);
            const auto* segm2 = dynamic_cast<const Part::GeomLineSegment*>(geo2);

            if (segm1 && segm2) {

                Base::Vector3d dir1 = segm1->getEndPoint() - segm1->getStartPoint();
                Base::Vector3d dir2 = segm2->getEndPoint() - segm2->getStartPoint();

                if ((checkVertical(dir1, angleprecision) || checkHorizontal(dir1, angleprecision))
                    && (checkVertical(dir2, angleprecision) || checkHorizontal(dir2, angleprecision))) {
                    // this is a job for horizontal/vertical constraints alone
                    continue;
                }
            }

            try {
                double u1 {};
                double u2 {};

                curve1->closestParameter(vc.v, u1);
                curve2->closestParameter(vc.v, u2);

                Base::Vector3d tgv1 = curve1->firstDerivativeAtParameter(u1).Normalize();
                Base::Vector3d tgv2 = curve2->firstDerivativeAtParameter(u2).Normalize();

                if (fabs(tgv1 * tgv2) > fabs(cos(angleprecision))) {
                    vc.Type = Sketcher::Tangent;
                }
                else if (fabs(tgv1 * tgv2) < fabs(cos(std::numbers::pi / 2 - angleprecision))) {
                    vc.Type = Sketcher::Perpendicular;
                }
            }
            catch (Base::Exception&) {
                Base::Console().warning(
                    "Point-On-Point Coincidence analysis: unable to obtain "
                    "derivative. Detection ignored.\n"
                );
                continue;
            }
        }
    }
}

Sketcher::Constraint* SketchAnalysis::create(const ConstraintIds& id)
{
    auto c = new Sketcher::Constraint();
    c->Type = id.Type;
    c->First = id.First;
    c->Second = id.Second;
    c->FirstPos = id.FirstPos;
    c->SecondPos = id.SecondPos;
    return c;
}

void SketchAnalysis::solveSketch(const char* errorText)
{
    int status {};
    int dofs {};
    solvesketch(status, dofs, true);

    if (status == int(Solver::RedundantConstraints)) {
        sketch->autoRemoveRedundants(DeleteOption::NoFlag);

        solvesketch(status, dofs, false);
    }

    if (status) {
        THROWMT(Base::RuntimeError, errorText);
    }
}

void SketchAnalysis::makeConstraints(std::vector<ConstraintIds>& ids)
{
    std::vector<Sketcher::Constraint*> constr;
    constr.reserve(ids.size());
    for (const auto& it : ids) {
        auto c = create(it);
        constr.push_back(c);
    }

    sketch->addConstraints(constr);
    ids.clear();

    for (auto it : constr) {
        delete it;
    }
}

void SketchAnalysis::makeConstraintsOneByOne(std::vector<ConstraintIds>& ids, const char* errorText)
{
    for (const auto& it : ids) {
        auto c = create(it);

        // addConstraint() creates a clone
        sketch->addConstraint(c);
        delete c;

        solveSketch(errorText);
    }

    ids.clear();
}

void SketchAnalysis::makeMissingPointOnPointCoincident()
{
    makeConstraints(vertexConstraints);
}

void SketchAnalysis::makeMissingPointOnPointCoincidentOneByOne()
{
    makeConstraintsOneByOne(
        vertexConstraints,
        QT_TRANSLATE_NOOP(
            "Exceptions",
            "Autoconstraint error: Unsolvable sketch while "
            "applying coincident constraints."
        )
    );
}

int SketchAnalysis::detectMissingVerticalHorizontalConstraints(double angleprecision)
{
    const std::vector<Part::Geometry*>& geom = sketch->getInternalGeometry();

    verthorizConstraints.clear();

    for (std::size_t i = 0; i < geom.size(); i++) {
        Part::Geometry* g = geom[i];

        if (const auto* segm = dynamic_cast<const Part::GeomLineSegment*>(g)) {
            Base::Vector3d dir = segm->getEndPoint() - segm->getStartPoint();

            ConstraintIds id;

            id.v = dir;
            id.First = (int)i;
            id.FirstPos = Sketcher::PointPos::none;
            id.Second = GeoEnum::GeoUndef;
            id.SecondPos = Sketcher::PointPos::none;

            if (checkVertical(dir, angleprecision)) {
                id.Type = Sketcher::Vertical;
                verthorizConstraints.push_back(id);
            }
            else if (checkHorizontal(dir, angleprecision)) {
                id.Type = Sketcher::Horizontal;
                verthorizConstraints.push_back(id);
            }
        }
    }

    return int(verthorizConstraints.size());
}

void SketchAnalysis::makeMissingVerticalHorizontal()
{
    makeConstraints(verthorizConstraints);
}

void SketchAnalysis::makeMissingVerticalHorizontalOneByOne()
{
    makeConstraintsOneByOne(
        verthorizConstraints,
        QT_TRANSLATE_NOOP(
            "Exceptions",
            "Autoconstraint error: Unsolvable sketch while "
            "applying vertical/horizontal constraints."
        )
    );
}

bool SketchAnalysis::checkVertical(Base::Vector3d dir, double angleprecision)
{
    return (dir.x == 0. && dir.y != 0.)
        || (fabs(dir.y / dir.x) > tan(std::numbers::pi / 2 - angleprecision));
}

bool SketchAnalysis::checkHorizontal(Base::Vector3d dir, double angleprecision)
{
    return (dir.y == 0. && dir.x != 0.) || (fabs(dir.x / dir.y) > (1 / tan(angleprecision)));
}

int SketchAnalysis::detectMissingEqualityConstraints(double precision)
{
    EqualityConstraints equalConstr;

    const std::vector<Part::Geometry*>& geom = sketch->getInternalGeometry();
    for (std::size_t i = 0; i < geom.size(); i++) {
        Part::Geometry* g = geom[i];
        equalConstr.addGeometry(g, int(i));
    }

    std::list<ConstraintIds> equallines = equalConstr.getEqualLines(precision);
    std::list<ConstraintIds> equalradius = equalConstr.getEqualRadius(precision);

    // Go through the available 'Coincident', 'Tangent' or 'Perpendicular' constraints
    // and check which of them is forcing two vertexes to be coincident.
    // If there is none but two vertexes can be considered equal a coincident constraint is missing.
    std::vector<Sketcher::Constraint*> constraint = sketch->Constraints.getValues();
    for (auto it : constraint) {
        if (it->Type == Sketcher::Equal) {
            ConstraintIds
                id {Base::Vector3d {}, it->First, it->Second, it->FirstPos, it->SecondPos, it->Type};

            auto pos = std::find_if(equallines.begin(), equallines.end(), Constraint_Equal(id));

            if (pos != equallines.end()) {
                equallines.erase(pos);
            }

            pos = std::find_if(equalradius.begin(), equalradius.end(), Constraint_Equal(id));

            if (pos != equalradius.end()) {
                equalradius.erase(pos);
            }
        }
    }

    this->lineequalityConstraints.clear();
    this->lineequalityConstraints.reserve(equallines.size());

    for (const auto& it : equallines) {
        this->lineequalityConstraints.push_back(it);
    }

    this->radiusequalityConstraints.clear();
    this->radiusequalityConstraints.reserve(equalradius.size());

    for (const auto& it : equalradius) {
        this->radiusequalityConstraints.push_back(it);
    }

    return int(this->lineequalityConstraints.size() + this->radiusequalityConstraints.size());
}

void SketchAnalysis::makeMissingEquality()
{
    std::vector<Sketcher::ConstraintIds> equalities(lineequalityConstraints);
    equalities.insert(
        equalities.end(),
        radiusequalityConstraints.begin(),
        radiusequalityConstraints.end()
    );
    makeConstraints(equalities);

    lineequalityConstraints.clear();
    radiusequalityConstraints.clear();
}

void SketchAnalysis::makeMissingEqualityOneByOne()
{
    std::vector<Sketcher::ConstraintIds> equalities(lineequalityConstraints);
    equalities.insert(
        equalities.end(),
        radiusequalityConstraints.begin(),
        radiusequalityConstraints.end()
    );

    makeConstraintsOneByOne(
        equalities,
        QT_TRANSLATE_NOOP(
            "Exceptions",
            "Autoconstraint error: Unsolvable sketch while "
            "applying equality constraints."
        )
    );
    lineequalityConstraints.clear();
    radiusequalityConstraints.clear();
}

void SketchAnalysis::solvesketch(int& status, int& dofs, bool updategeo)
{
    status = sketch->solve(updategeo);

    if (updategeo) {
        dofs = sketch->setUpSketch();
    }
    else {
        dofs = sketch->getLastDoF();
    }

    if (sketch->getLastHasRedundancies()) {
        status = int(Solver::RedundantConstraints);
    }

    if (dofs < 0) {
        status = int(Solver::OverConstrained);
    }
    else if (sketch->getLastHasConflicts()) {
        status = int(Solver::ConflictingConstraints);
    }
}

void SketchAnalysis::autoDeleteAllConstraints()
{
    App::Document* doc = sketch->getDocument();
    doc->openTransaction("delete all constraints");
    // We start from zero
    sketch->deleteAllConstraints();

    doc->commitTransaction();

    // a failure should not be possible at this moment as we start from a clean situation
    solveSketch(
        QT_TRANSLATE_NOOP("Exceptions", "Autoconstraint error: Unsolvable sketch without constraints.")
    );
}

void SketchAnalysis::autoHorizontalVerticalConstraints()
{
    App::Document* doc = sketch->getDocument();
    doc->openTransaction("add vertical/horizontal constraints");

    makeMissingVerticalHorizontal();

    // finish the transaction and update
    doc->commitTransaction();

    solveSketch(QT_TRANSLATE_NOOP(
        "Exceptions",
        "Autoconstraint error: Unsolvable sketch after applying "
        "horizontal and vertical constraints."
    ));
}

void SketchAnalysis::autoPointOnPointCoincident()
{
    App::Document* doc = sketch->getDocument();
    doc->openTransaction("add coincident constraint");

    makeMissingPointOnPointCoincident();

    // finish the transaction and update
    doc->commitTransaction();

    solveSketch(QT_TRANSLATE_NOOP(
        "Exceptions",
        "Autoconstraint error: Unsolvable sketch after applying "
        "point-on-point constraints."
    ));
}

void SketchAnalysis::autoMissingEquality()
{
    App::Document* doc = sketch->getDocument();
    doc->openTransaction("add equality constraints");

    try {
        makeMissingEquality();
    }
    catch (Base::RuntimeError&) {
        doc->abortTransaction();
        throw;
    }

    // finish the transaction and update
    doc->commitTransaction();

    solveSketch(QT_TRANSLATE_NOOP(
        "Exceptions",
        "Autoconstraint error: Unsolvable sketch after "
        "applying equality constraints."
    ));
}

int SketchAnalysis::autoconstraint(double precision, double angleprecision, bool includeconstruction)
{
    autoDeleteAllConstraints();

    // STAGE 1: Vertical/Horizontal Line Segments
    int nhv = detectMissingVerticalHorizontalConstraints(angleprecision);

    // STAGE 2: Point-on-Point constraint (Coincidents, endpoint perp, endpoint tangency)
    // Note: We do not apply the vertical/horizontal constraints before calculating the pointonpoint
    // constraints
    //       as the solver may move the geometry in the meantime and prevent correct detection
    int nc = detectMissingPointOnPointConstraints(precision, includeconstruction);

    if (nc > 0) {  // STAGE 2a: Classify point-on-point into coincidents, endpoint perp, endpoint
                   // tangency
        analyseMissingPointOnPointCoincident(angleprecision);
    }

    // STAGE 3: Equality constraint detection
    int ne = detectMissingEqualityConstraints(precision);

    Base::Console().log(
        "Constraints: Vertical/Horizontal: %d found. "
        "Point-on-point: %d. Equality: %d\n",
        nhv,
        nc,
        ne
    );

    // Applying STAGE 1, if any
    if (nhv > 0) {
        autoHorizontalVerticalConstraints();
    }

    // Applying STAGE 2
    if (nc > 0) {
        autoPointOnPointCoincident();
    }

    // Applying STAGE 3
    if (ne > 0) {
        autoMissingEquality();
    }

    return 0;
}


std::vector<Base::Vector3d> SketchAnalysis::getOpenVertices() const
{
    std::vector<Base::Vector3d> points;
    TopoDS_Shape shape = sketch->Shape.getValue();

    Base::Placement Plm = sketch->Placement.getValue();

    Base::Placement invPlm = Plm.inverse();

    // build up map vertex->edge
    TopTools_IndexedDataMapOfShapeListOfShape vertex2Edge;
    TopExp::MapShapesAndAncestors(shape, TopAbs_VERTEX, TopAbs_EDGE, vertex2Edge);
    for (int i = 1; i <= vertex2Edge.Extent(); ++i) {
        const TopTools_ListOfShape& los = vertex2Edge.FindFromIndex(i);
        if (los.Extent() != 2) {
            const TopoDS_Vertex& vertex = TopoDS::Vertex(vertex2Edge.FindKey(i));
            gp_Pnt pnt = BRep_Tool::Pnt(vertex);
            Base::Vector3d pos;
            invPlm.multVec(Base::Vector3d(pnt.X(), pnt.Y(), pnt.Z()), pos);
            points.push_back(pos);
        }
    }

    return points;
}

SketchAnalysis::SketchAnalysisResult SketchAnalysis::getDiagnosticReport(double tolerance) const
{
    SketchAnalysisResult report;

    // 1. Force a refresh of the Sketch state (Public alternative to recompute)
    auto* mutableSketch = const_cast<Sketcher::SketchObject*>(sketch);
    mutableSketch->execute();

    // 2. Identify Internal Degenerate Geometry
    const std::vector<Part::Geometry*>& internalGeom = sketch->getInternalGeometry();
    for (size_t i = 0; i < internalGeom.size(); ++i) {
        if (!internalGeom[i]) {
            report.internalDegenerate.insert(static_cast<int>(i));
            continue;
        }

        // Use Facade to check if it's a real (non-construction) line
        std::unique_ptr<Sketcher::GeometryFacade> gf(
            Sketcher::GeometryFacade::getFacade(internalGeom[i])
        );
        if (gf && !gf->getConstruction()) {
            if (auto curve = dynamic_cast<const Part::GeomCurve*>(internalGeom[i])) {
                try {
                    // Check length against tolerance
                    if (curve->length(curve->getFirstParameter(), curve->getLastParameter())
                        < tolerance) {
                        report.internalDegenerate.insert(static_cast<int>(i));
                    }
                }
                catch (...) {
                    report.internalDegenerate.insert(static_cast<int>(i));
                }
            }
        }
    }

    // 3. External Geometry Analysis (Fixed Type Mismatch) ---
    // Use ExternalGeometry (PropertyLinkSubList) for the raw link objects
    const std::vector<App::DocumentObject*>& linkedObjs = sketch->ExternalGeometry.getValues();
    const std::vector<Part::Geometry*>& resolvedGeom = sketch->getExternalGeometry();

    for (int j = 0; j < static_cast<int>(linkedObjs.size()); j++) {
        bool isBroken = false;

        // CASE 1: The target object itself is null (it was deleted)
        if (linkedObjs[j] == nullptr) {
            isBroken = true;
        }
        // CASE 2: The sub-element (e.g., #c:3) is missing from the object
        // If the index exists in the property but not in the resolver, it's a "zombie" link.
        else if (j >= static_cast<int>(resolvedGeom.size()) || resolvedGeom[j] == nullptr) {
            isBroken = true;
        }

        if (isBroken) {
            // This index j matches the "Missing_External_Geo" error in your console
            report.externalBroken.insert(j);

            // Log the specific object name for confirmation
            std::string objName = linkedObjs[j] ? linkedObjs[j]->getNameInDocument()
                                                : "Deleted Object";
            Base::Console().warning(("Diagnostic: Captured broken link at index "
                                     + std::to_string(j) + " to " + objName + "\n")
                                        .c_str());
        }
    }

    // 2. Access the internal geometry list (ExternalGeo contains the Part::Geometry objects)
    const std::vector<Part::Geometry*>& geoms = sketch->ExternalGeo.getValues();

    for (size_t i = 0; i < geoms.size(); ++i) {
        if (!geoms[i]) {
            continue;
        }

        // Use the Facade logic from SketchObject.cpp
        auto egf = Sketcher::ExternalGeometryFacade::getFacade(geoms[i]);
        if (!egf) {
            continue;
        }

        // Check the 'Missing' flag set during SketchObject::execute()
        // This flag is what triggers the "missing reference: Pad" console error
        if (egf->testFlag(Sketcher::ExternalGeometryExtension::Missing)) {
            report.externalBroken.insert(static_cast<int>(i));

            // Console confirmation matching the error trace you provided
            Base::Console().warning(("Diagnostic caught: " + egf->getRef() + " is missing\n").c_str());
        }
        else {
            // Check for geometric degeneracy if the reference is NOT missing
            if (auto curve = dynamic_cast<const Part::GeomCurve*>(geoms[i])) {
                try {
                    if (curve->length(curve->getFirstParameter(), curve->getLastParameter())
                        < tolerance) {
                        report.externalBroken.insert(static_cast<int>(i));
                    }
                }
                catch (...) {
                    report.externalBroken.insert(static_cast<int>(i));
                }
            }
        }
    }

    return report;
}


int SketchAnalysis::detectDegeneratedGeometries(double tolerance) const
{
    // 1. Get the comprehensive report which includes the 'Missing_External_Geo' check
    SketchAnalysis::SketchAnalysisResult report = getDiagnosticReport(tolerance);

    // 2. The total count now includes broken purple references AND tiny internal lines
    int internalCount = static_cast<int>(report.internalDegenerate.size());
    int externalBrokenCount = static_cast<int>(report.externalBroken.size());

    return internalCount + externalBrokenCount;
}

int SketchAnalysis::removeDegeneratedGeometries(double tolerance)
{
    int removedCount = 0;
    SketchAnalysisResult report = getDiagnosticReport(tolerance);

    if (report.internalDegenerate.empty() && report.externalBroken.empty()) {
        return 0;
    }

    App::Document* doc = sketch->getDocument();
    if (doc) {
        doc->openTransaction("Convert and Clean Broken External Links");

        const std::vector<Part::Geometry*>& externalGeom = sketch->getExternalGeometry();

        // 1. RESCUE PHASE: Convert broken links to INTERNAL CONSTRUCTION lines
        for (int extIdx : report.externalBroken) {

            // Only rescue if there is actual geometry data left to copy
            if (extIdx < static_cast<int>(externalGeom.size()) && externalGeom[extIdx] != nullptr) {

                std::unique_ptr<GeometryFacade> gf(GeometryFacade::getFacade(externalGeom[extIdx]));
                bool shouldRescue = true;

                if (gf && gf->getGeometry()) {
                    if (auto curve = dynamic_cast<const Part::GeomCurve*>(gf->getGeometry())) {
                        try {
                            if (curve->length(curve->getFirstParameter(), curve->getLastParameter())
                                < tolerance) {
                                shouldRescue = false;
                            }
                        }
                        catch (...) {
                            shouldRescue = false;
                        }
                    }
                }
                else {
                    shouldRescue = false;
                }

                if (shouldRescue) {
                    // 1. Create a deep copy of the geometry
                    Part::Geometry* copy = externalGeom[extIdx]->copy();

                    // 2. Identify and delete the ExternalGeometryExtension
                    // In FreeCAD, extensions are often stored in a map; deleteExtension takes the
                    // type ID
                    copy->deleteExtension(Sketcher::ExternalGeometryExtension::getClassTypeId());

                    // 3. Add as internal construction (orange)
                    // Now that the extension is gone, 'true' will work as intended
                    sketch->addGeometry(copy, true);

                    Base::Console().warning(
                        "Diagnostic: Successfully converted broken external "
                        "link to internal construction.\n"
                    );
                }
            }

            // 2. CONSTRAINT CLEANUP: Unbind constraints from the dead external GeoID
            int targetGeoId = -(extIdx + 1);
            const std::vector<Sketcher::Constraint*>& constraints = sketch->Constraints.getValues();
            for (int k = static_cast<int>(constraints.size()) - 1; k >= 0; --k) {
                if (constraints[k]->First == targetGeoId || constraints[k]->Second == targetGeoId
                    || constraints[k]->Third == targetGeoId) {
                    sketch->delConstraint(k);
                }
            }
        }

        // 3. SEVER PHASE: Delete the original external links from the property list
        // Iterating backwards is mandatory here to keep indices stable.
        // We use the indices stored in report.externalBroken which were identified
        // in the diagnostic phase (including those with the Missing flag).
        auto* mutableSketch = const_cast<Sketcher::SketchObject*>(sketch);

        // A. Identify and delete constraints referencing broken external geometry
        const std::vector<Sketcher::Constraint*>& constrs = mutableSketch->Constraints.getValues();
        std::vector<int> constraintIndicesToDelete;

        for (int i = 0; i < static_cast<int>(constrs.size()); ++i) {
            Sketcher::Constraint* c = constrs[i];
            if (!c) {
                continue;
            }

            // A reference is broken if its index is in our report.externalBroken set
            // In Sketcher, external indices are typically negative (-1, -2, etc.)
            auto isRefBroken = [&](int geoIdx) {
                if (geoIdx < 0) {
                    int extIdx = std::abs(geoIdx) - 1;
                    return report.externalBroken.count(extIdx) > 0;
                }
                return false;
            };

            if (isRefBroken(c->First) || isRefBroken(c->Second) || isRefBroken(c->Third)) {
                constraintIndicesToDelete.push_back(i);
            }
        }

        // B. Delete constraints in REVERSE order to maintain index stability
        for (auto it = constraintIndicesToDelete.rbegin(); it != constraintIndicesToDelete.rend();
             ++it) {
            mutableSketch->delConstraint(*it);
        }

        // C. Now delete the external geometry itself (also in REVERSE)
        for (auto it = report.externalBroken.rbegin(); it != report.externalBroken.rend(); ++it) {
            mutableSketch->delExternal(*it);
        }

        // 4. CLEANUP PHASE: Remove tiny internal lines
        std::vector<int> sortedInternal(
            report.internalDegenerate.begin(),
            report.internalDegenerate.end()
        );
        for (auto it = sortedInternal.rbegin(); it != sortedInternal.rend(); ++it) {
            sketch->delGeometry(*it);
            removedCount++;
        }

        // 5. FORCE CACHE SYNC
        // Directly touching the 'ExternalGeo' property is sometimes needed
        // if delExternal is leaving 'ghosts'
        mutableSketch->ExternalGeo.setSize(mutableSketch->getExternalGeometryCount());

        // 6. COMMIT AND EXECUTE
        doc->commitTransaction();
        mutableSketch->execute();  // This forces the solver to realize the links are gone

        sketch->solve();

        // Finalize the changes in the Document
        sketch->touch();
        doc->recompute();
    }

    return removedCount;
}
