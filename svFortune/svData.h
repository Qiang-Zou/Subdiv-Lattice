
#ifndef SphericalVoronoi_svdata_h
#define SphericalVoronoi_svdata_h

#include <memory>
#include <vector>
#include <cassert>
#include <set>
#include <ostream>
#include <iterator>
#include <algorithm>
#include <glm/glm.hpp>
#include <string>

namespace sv
{
    class half_edge;
    class cell;
    class vertex;
    class beach_arc;
    class site_event;
    class circle_event;

    typedef double Real;
    typedef glm::dvec3 Real3;
    typedef std::shared_ptr<vertex> vertex_ptr;
    typedef std::shared_ptr<half_edge> half_edge_ptr;
    typedef std::shared_ptr<cell> cell_ptr;
    typedef std::shared_ptr<beach_arc> beach_arc_ptr;
    typedef std::shared_ptr<circle_event> circle_event_ptr;

    class Point
    {
    public:
        Point() : theta(0), phi(0) { computePosition(); }

        Point(Real theta_, Real phi_)
            : theta(theta_), phi(phi_)
        {
            computePosition();
        }

        Point(const Real3 &direction)
        {
            assignDirection(direction);
        }

        Point(Real x, Real y, Real z)
        {
            assignDirection(Real3(x, y, z));
        }

        Real theta;
        Real phi;
        Real3 position;

        void assignDirection(const Real3 &direction)
        {
            Real r = glm::length(direction);
            assert(r > 0);
            theta = glm::acos(glm::clamp<Real>(direction.z / r, -1.0, 1.0));
            phi = glm::atan(direction.y, direction.x);
            position = direction / r;
        }

        bool equalWithEps(const Point &p2, float eps) const
        {
            using namespace glm;
            return abs(position.x - p2.position.x) < eps &&
                   abs(position.y - p2.position.y) < eps &&
                   abs(position.z - p2.position.z) < eps;
        }

        bool operator<(const Point &p2)
        {
            return (theta < p2.theta) || (theta == p2.theta && phi < p2.phi);
        }

    private:
        void computePosition()
        {
            using namespace glm;
            position = Real3(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
        }
    };

    class SphericalLine
    {
    public:
        SphericalLine() : direction(Real3(0, 0, 1)), xi(0) {}
        SphericalLine(const Real3 &direction_, Real xi_) : direction(glm::normalize(direction_)), xi(xi_) {}

        Real3 direction;
        Real xi;
    };

    class cell
    {
    public:
        cell(int i, const Point &p) : index(i), point(p)
        {
        }
        int index;
        Point point;

        std::vector<half_edge_ptr> halfEdges;
        std::vector<vertex_ptr> corners;

        void reset()
        {
            halfEdges.clear();
        }
    };

    class vertex
    {
    public:
        vertex(const Point &p, std::shared_ptr<cell> c0, std::shared_ptr<cell> c1) : point(p)
        {
            // cells.insert(c0);
            // cells.insert(c1);
        }
        vertex(const Point &p, std::shared_ptr<cell> c0, std::shared_ptr<cell> c1, std::shared_ptr<cell> c2) : point(p)
        {
            cells.insert(c0);
            cells.insert(c1);
            cells.insert(c2);
        }

        void reset()
        {
            halfEdges.clear();
            cells.clear();
        }

        uint32_t index;
        Point point;
        std::vector<half_edge_ptr> halfEdges;
        std::set<cell_ptr> cells;
        std::string type;
    };

    class half_edge
    {
    public:
        half_edge(std::shared_ptr<vertex> s, std::shared_ptr<vertex> e)
            : start(s), end(e)
        {
        }
        uint32_t index;
        cell_ptr cell;
        cell_ptr otherCell;
        vertex_ptr start;
        vertex_ptr end;
        half_edge_ptr prev;
        half_edge_ptr next;
        half_edge_ptr twin;

        void reset()
        {
            cell.reset();
            start.reset();
            end.reset();
            prev.reset();
            next.reset();
            twin.reset();
        }
    };

    class beach_arc
    {
    public:
        beach_arc(std::shared_ptr<cell> cell_)
            : cell(cell_)
        {
        }

        std::shared_ptr<cell> cell;

        std::shared_ptr<circle_event> circleEvent = nullptr; // the related circle event

        std::shared_ptr<vertex> startVertex;

        bool operator<(const beach_arc &ba) const
        {
            return cell->point.phi < ba.cell->point.phi;
        }
    };

    class site_event
    {
    public:
        site_event(std::shared_ptr<cell> cell_)
            : cell(cell_)
        {
            theta = cell->point.theta;
            phi = cell->point.phi;
        }

        std::shared_ptr<cell> cell;
        Real theta;
        Real phi;

        bool operator<(const site_event &right) const
        {
            return (theta < right.theta) || (theta == right.theta && phi < right.phi);
        }
    };

    class circle_event
    {
    public:
        circle_event(const beach_arc_ptr &arc_i_, const beach_arc_ptr &arc_j_, const beach_arc_ptr &arc_k_)
            : arc_i(arc_i_), arc_j(arc_j_), arc_k(arc_k_)
        {
            using namespace glm;

            auto pij = cell_i()->point.position - cell_j()->point.position;
            auto pkj = cell_k()->point.position - cell_j()->point.position;
            auto direction = cross(pij, pkj);
            circle_center = Point(direction);
            circle_radius = acos(dot(circle_center.position, cell_i()->point.position));
            theta = acos(circle_center.position.z) + circle_radius;
        }

        beach_arc_ptr arc_i;
        beach_arc_ptr arc_j;
        beach_arc_ptr arc_k;

        cell_ptr cell_i() const { return arc_i->cell; }
        cell_ptr cell_j() const { return arc_j->cell; }
        cell_ptr cell_k() const { return arc_k->cell; }

        Point circle_center;
        Real circle_radius;

        Real theta; // the lowest point on circle

        bool operator<(const circle_event &ce) const
        {
            return ce.theta - theta > 1e-6;
        }
    };

    struct compare_circle_event_priority
    {
        bool operator()(const std::shared_ptr<circle_event> &left, const std::shared_ptr<circle_event> &right) const
        {
            return *left < *right;
        }
    };

}

#endif
