
#ifndef SphericalVoronoi_svvoronoi_h
#define SphericalVoronoi_svvoronoi_h

#include "svData.h"
#include <functional>
#include <iostream>
#include <gp_Pnt.hxx>

namespace sv
{
    class SphericalVoronoiCore
    {
    public:
        SphericalVoronoiCore(const std::vector<Real3> &directions);

        bool isFinished() const;
        void step(Real maxDeltaXi);
        void solve(std::function<void(int)> cb = nullptr); // step until finished

        const std::vector<half_edge_ptr> &getHalfEdges() const { return halfEdges; }
        const std::vector<vertex_ptr> &getVertices() const { return vertices; }
        const std::vector<cell_ptr> &getCells() const { return cells; }
        std::vector<vertex_ptr> deleteVertices;

    protected:
        SphericalLine scanLine;

        constexpr static Real eps = 1e-5;

        std::vector<half_edge_ptr> halfEdges;
        std::vector<vertex_ptr> vertices;
        std::vector<cell_ptr> cells;

        typedef std::vector<beach_arc_ptr> beach_line;
        beach_line beach;
        std::vector<site_event> siteEventQueue;
        std::vector<circle_event_ptr> circleEventQueue;

        beach_line::const_iterator getPrevArcOnBeach(beach_line::const_iterator it) const;
        beach_line::const_iterator getNextArcOnBeach(beach_line::const_iterator it) const;
        bool isArcOnBeach(const beach_arc_ptr &arc) const;

        void addNewSiteEvent(const site_event &event);

        void addNewCircleEvent(const std::shared_ptr<circle_event> &event);
        void removeCircleEvent(const std::shared_ptr<circle_event> &event);

        bool intersectWithNextArc(beach_line::const_iterator itArc, Real xi, Point &oPoint) const;
        bool intersectWithPrevArc(beach_line::const_iterator itArc, Real xi, Point &oPoint) const;
        void handleSiteEvent(site_event &event);
        void handleCircleEvent(const circle_event_ptr &event);

        static Point thetaToPoint(Real theta, bool positive, Real xi, Real theta1, Real phi1);
        static Point phiToPoint(Real phi, Real xi, Real theta1, Real phi1);
        static bool arcsIntersection(const beach_arc &arc1, const beach_arc &arc2, Real xi, Point &oPoint);

        void finializeGraph();
        void cleanupMiddleVertices();
        void duplicateHalfEdges();
        void bindHalfEdgesToCells();
    };
}

#endif
