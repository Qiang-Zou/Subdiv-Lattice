#include "TopoMesh.h"
#include "nodeManager.h"
#include <glm/gtc/matrix_transform.hpp>

TopoMesh::TopoMesh(node& _node) : latNode(_node)
{
}

void TopoMesh::buildInitialMesh()
{
    if (latNode.circles.size() == 3)
    {
        voronoi_vertices_belong_cell.resize(50 * 3);
        double radii = latNode.sphereRadii;
        gp_Vec v1(latNode.origin, latNode.circles[0].circle_center);
        v1.Normalize();
        gp_Vec v2(latNode.origin, latNode.circles[1].circle_center);
        v2.Normalize();
        gp_Vec v3(latNode.origin, latNode.circles[2].circle_center);
        v3.Normalize();
        gp_Vec v4 = v1 + v2 + v3;
        // v4 = gp_Vec(0, 0, 1);
        v4.Normalize();
        gp_Pnt p1(latNode.origin.X() + v4.X() * radii, latNode.origin.Y() + v4.Y() * radii, latNode.origin.Z() + v4.Z() * radii);
        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(0);
        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(1);
        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(2);
        voronoi_vertices.push_back(p1);

        gp_Pnt p2(latNode.origin.X() - v4.X() * radii, latNode.origin.Y() - v4.Y() * radii, latNode.origin.Z() - v4.Z() * radii);
        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(0);
        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(1);
        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(2);
        voronoi_vertices.push_back(p2);

        std::vector<int> vcell_1, vcell_2, vcell_3;
        gp_Vec v01(v1 + v2);
        v01.Normalize();
        gp_Vec v02(v1 + v3);
        v02.Normalize();
        gp_Vec v12(v2 + v3);
        v12.Normalize();

        gp_Pnt p01(latNode.origin.X() + v01.X() * radii, latNode.origin.Y() + v01.Y() * radii, latNode.origin.Z() + v01.Z() * radii);
        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(0);
        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(1);
        voronoi_vertices.push_back(p01);

        double a = 0.5, b = 1 - a;
        gp_Pnt p101(p1.X() * a + p01.X() * b, p1.Y() * a + p01.Y() * b, p1.Z() * a + p01.Z() * b);
        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(0);
        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(1);
        voronoi_vertices.push_back(p101);

        gp_Pnt p201(p2.X() * a + p01.X() * b, p2.Y() * a + p01.Y() * b, p2.Z() * a + p01.Z() * b);
        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(0);
        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(1);
        voronoi_vertices.push_back(p201);

        gp_Pnt p02(latNode.origin.X() + v02.X() * radii, latNode.origin.Y() + v02.Y() * radii, latNode.origin.Z() + v02.Z() * radii);
        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(0);
        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(2);
        voronoi_vertices.push_back(p02);

        gp_Pnt p102(p1.X() * a + p02.X() * b, p1.Y() * a + p02.Y() * b, p1.Z() * a + p02.Z() * b);
        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(0);
        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(2);
        voronoi_vertices.push_back(p102);

        gp_Pnt p202(p2.X() * a + p02.X() * b, p2.Y() * a + p02.Y() * b, p2.Z() * a + p02.Z() * b);
        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(0);
        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(2);
        voronoi_vertices.push_back(p202);

        gp_Pnt p12(latNode.origin.X() + v12.X() * radii, latNode.origin.Y() + v12.Y() * radii, latNode.origin.Z() + v12.Z() * radii);
        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(1);
        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(2);
        voronoi_vertices.push_back(p12);

        gp_Pnt p112(p1.X() * a + p12.X() * b, p1.Y() * a + p12.Y() * b, p1.Z() * a + p12.Z() * b);
        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(1);
        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(2);
        voronoi_vertices.push_back(p112);

        gp_Pnt p212(p2.X() * a + p12.X() * b, p2.Y() * a + p12.Y() * b, p2.Z() * a + p12.Z() * b);
        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(1);
        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(2);
        voronoi_vertices.push_back(p212);

        // p1-0, p2-1, p01-2, p101-3, p201-4 p02-5, p102-6, p202-7, p12-8, p112-9, p212-10

        vcell_1.push_back(0);
        vcell_1.push_back(6);
        vcell_1.push_back(5);
        vcell_1.push_back(7);
        vcell_1.push_back(1);
        vcell_1.push_back(4);
        vcell_1.push_back(2);
        vcell_1.push_back(3);
        voronoi_cells.push_back(vcell_1);

        vcell_2.push_back(0);
        vcell_2.push_back(3);
        vcell_2.push_back(2);
        vcell_2.push_back(4);
        vcell_2.push_back(1);
        vcell_2.push_back(10);
        vcell_2.push_back(8);
        vcell_2.push_back(9);
        voronoi_cells.push_back(vcell_2);

        vcell_3.push_back(0);
        vcell_3.push_back(9);
        vcell_3.push_back(8);
        vcell_3.push_back(10);
        vcell_3.push_back(1);
        vcell_3.push_back(7);
        vcell_3.push_back(5);
        vcell_3.push_back(6);
        voronoi_cells.push_back(vcell_3);
    }
    else
    {
        FortuneVoronoi();
        removeDuplicateVertices();
        addVtxInEdge_all();
    }

    proVronoiVtxToNode();
    meshFromVoronoiDiagram_irregularHandle();
}

void TopoMesh::FortuneVoronoi()
{
    voronoi_cells.clear();
    voronoi_vertices.clear();
    voronoi_vertices_belong_cell.clear();
    voronoi_vertices_belong_cell.resize(50 * latNode.circles.size());
    std::vector<sv::Real3> directions;
    std::vector<sv::Point> pnts;
    for (int k = 0; k < latNode.circles.size(); k++)
    {
        gp_Vec vec(latNode.circles[k].circle_center.X() - latNode.origin.X(), latNode.circles[k].circle_center.Y() - latNode.origin.Y(), latNode.circles[k].circle_center.Z() - latNode.origin.Z());
        vec.Normalize();
        sv::Real3 p1(vec.X(), vec.Y(), vec.Z());
        sv::Point p(p1);
        pnts.push_back(p);
        directions.push_back(p1);
    }

    bool hasThetaEqu = checkTheatEqu(pnts);
    std::vector<sv::Real3> points = directions;
    int times = 1;
    float rotateTheta = 0;
    float rotateZ = 0;
    float rotateY = 0;
    float rotateX = 0;
    while (hasThetaEqu)
    {
        pnts.clear();
        points.clear();

        if (times < 10)
        {
            for (auto dir : directions)
            {
                glm::mat4 trans = glm::mat4(1.0f);
                trans = glm::rotate(trans, glm::radians(5.0f * times), glm::vec3(1.0, 0.0, 0.0));
                rotateTheta = 5.0 * times;
                glm::vec4 vec(dir.x, dir.y, dir.z, 1.0f);
                vec = trans * vec;
                sv::Real3 p2(vec.x, vec.y, vec.z);
                sv::Point p(p2);
                pnts.push_back(p);
                points.push_back(p2);
            }
            hasThetaEqu = checkTheatEqu(pnts);
            times++;
        }
        else
        {
            for (auto dir : directions)
            {
                glm::mat4 trans = glm::mat4(1.0f);
                trans = glm::rotate(trans, glm::radians(10.f), glm::vec3(1.0, 1.0, 0.0));
                rotateY = 10.0;
                glm::vec4 vec(dir.x, dir.y, dir.z, 1.0f);
                vec = trans * vec;

                glm::mat4 trans1 = glm::mat4(1.0f);
                trans1 = glm::rotate(trans1, glm::radians(10.f), glm::vec3(0.0, 1.0, 1.0));
                rotateZ = 10.0;
                vec = trans1 * vec;

                glm::mat4 trans2 = glm::mat4(1.0f);
                trans2 = glm::rotate(trans2, glm::radians(10.f), glm::vec3(1.0, 1.0, 1.0));
                rotateX = 10.0;
                vec = trans2 * vec;

                sv::Real3 p2(vec.x, vec.y, vec.z);
                sv::Point p(p2);
                pnts.push_back(p);
                points.push_back(p2);
            }
            hasThetaEqu = checkTheatEqu(pnts);
            times++;
        }
        if (times > 20)
        {
            std::cout << "there are points on the same phi" << std::endl;
            assert(times <= 20);
        }
    }

    sv::SphericalVoronoiCore SphericalVoronoi(points);
    SphericalVoronoi.solve();

    auto cells = SphericalVoronoi.getCells();
    auto vertices = SphericalVoronoi.getVertices();
    auto edges = SphericalVoronoi.getHalfEdges();

    std::vector<int> corners;
    for (int j = 0; j < vertices.size(); j++)
    {
        if (vertices[j]->type == "circleCenter")
        {
            corners.push_back(j);
            for (auto iter = vertices[j]->cells.begin(); iter != vertices[j]->cells.end(); iter++)
            {
                sv::cell_ptr cell = *iter;
            }
        }
    }

    std::vector<gp_Pnt> verts;
    for (int j = 0; j < vertices.size(); j++)
    {
        gp_Pnt pnt;
        glm::vec4 vec;
        if (rotateY == 0)
        {
            pnt = gp_Pnt(vertices[j]->point.position.x, vertices[j]->point.position.y, vertices[j]->point.position.z);
            glm::mat4 trans = glm::mat4(1.0f);
            trans = glm::rotate(trans, glm::radians(-rotateTheta), glm::vec3(1.0, 0.0, 0.0));
            vec = glm::vec4(pnt.X(), pnt.Y(), pnt.Z(), 1.0f);
            vec = trans * vec;
        }
        else
        {
            pnt = gp_Pnt(vertices[j]->point.position.x, vertices[j]->point.position.y, vertices[j]->point.position.z);
            glm::mat4 trans = glm::mat4(1.0f);
            trans = glm::rotate(trans, glm::radians(-rotateX), glm::vec3(1.0, 1.0, 1.0));
            vec = glm::vec4(pnt.X(), pnt.Y(), pnt.Z(), 1.0f);
            vec = trans * vec;

            glm::mat4 trans1 = glm::mat4(1.0f);
            trans1 = glm::rotate(trans1, glm::radians(-rotateZ), glm::vec3(0.0, 1.0, 1.0));
            vec = trans1 * vec;

            glm::mat4 trans2 = glm::mat4(1.0f);
            trans2 = glm::rotate(trans2, glm::radians(-rotateY), glm::vec3(1.0, 1.0, 0.0));
            vec = trans2 * vec;
        }

        pnt.SetX(vec.x * latNode.sphereRadii);
        pnt.SetY(vec.y * latNode.sphereRadii);
        pnt.SetZ(vec.z * latNode.sphereRadii);
        // std::cout << pnt.X() << " " << pnt.Y() << " " << pnt.Z() << std::endl;
        verts.push_back(pnt);
    }

    std::vector<int> pairs(verts.size());
    for (int j = 0; j < corners.size(); j++)
    {
        std::vector<int> belongCell;
        voronoi_vertices.push_back(verts[corners[j]]);
        // voronoi_vertices[voronoi_vertices.size()-1].SetX(voronoi_vertices[voronoi_vertices.size()-1].X() + latNode.origin.X());
        // voronoi_vertices[voronoi_vertices.size()-1].SetY(voronoi_vertices[voronoi_vertices.size()-1].Y() + latNode.origin.Y());
        // voronoi_vertices[voronoi_vertices.size()-1].SetZ(voronoi_vertices[voronoi_vertices.size()-1].Z() + latNode.origin.Z());
        pairs[corners[j]] = j;
        // std::cout << j << ": " << voronoi_vertices[voronoi_vertices.size()-1].X() << " " <<voronoi_vertices[voronoi_vertices.size()-1].Y() << " " << voronoi_vertices[voronoi_vertices.size()-1].Z() << " belong cells: ";
        for (auto iter = vertices[corners[j]]->cells.begin(); iter != vertices[corners[j]]->cells.end(); ++iter)
        {
            sv::cell_ptr ce = *iter;
            belongCell.push_back(ce->index);
        }

        voronoi_vertices_belong_cell[j] = belongCell;
        // std::cout << std::endl;
    }
    voronoi_cells.resize(cells.size());
    for (int j = 0; j < cells.size(); j++)
    {
        std::vector<int> cell;
        for (int k = 0; k < cells[j]->corners.size(); k++)
            cell.push_back(pairs[cells[j]->corners[k]->index]);

        if (cell.size() < 3)
        {
            std::cout << "cell size()<3" << std::endl;
            assert(cell.size() > 2);
        }
        voronoi_cells[cells[j]->index] = cell;
    }

    for (int j = 0; j < voronoi_cells.size(); j++)
    {
        reorderPointsCCW(latNode.circles[j].circle_center, voronoi_cells[j], voronoi_vertices);
    }

    for (int j = 0; j < voronoi_vertices.size(); j++)
    {
        voronoi_vertices[j].SetX(voronoi_vertices[j].X() + latNode.origin.X());
        voronoi_vertices[j].SetY(voronoi_vertices[j].Y() + latNode.origin.Y());
        voronoi_vertices[j].SetZ(voronoi_vertices[j].Z() + latNode.origin.Z());
    }
}

void TopoMesh::reorderPointsCCW(const gp_Pnt& circleCenter, std::vector<int>& indexes, const std::vector<gp_Pnt>& points)
{
    double sumX = 0, sumY = 0, sumZ = 0;
    for (int i = 0; i < indexes.size(); i++)
    {
        sumX = sumX + points[indexes[i]].X();
        sumY = sumY + points[indexes[i]].Y();
        sumZ = sumZ + points[indexes[i]].Z();
    }
    sumX = sumX / indexes.size();
    sumY = sumY / indexes.size();
    sumZ = sumZ / indexes.size();

    gp_Pnt center(sumX, sumY, sumZ);
    gp_Pnt refPnt(points[indexes[0]].X(), points[indexes[0]].Y(), points[indexes[0]].Z());
    gp_Pnt origin(0, 0, 0);
    gp_Vec normal(origin, center);
    normal.Normalize();

    gp_Vec vec(origin, refPnt);
    double moveLen = vec.Dot(normal);
    gp_Pnt proj = origin.Translated(moveLen * normal);

    gp_Vec v1(proj, refPnt);
    gp_Vec v2(origin, center);

    gp_Lin line(gp_Pnt(0, 0, 0), gp_Dir(v2));

    gp_Vec z(gp_Pnt(0, 0, 0), center);
    z.Normalize();
    gp_Vec x(proj, refPnt);
    x.Normalize();
    gp_Vec y = z.Crossed(x);
    y.Normalize();

    Eigen::Matrix<double, 4, 4> trans, trans1;
    trans1 << x.X(), y.X(), z.X(), proj.X(),
        x.Y(), y.Y(), z.Y(), proj.Y(),
        x.Z(), y.Z(), z.Z(), proj.Z(),
        0, 0, 0, 1;
    trans = trans1.inverse();

    gp_Pnt planeOrigin(proj);
    gp_Dir planeNormal(z); // Plane's normal points along Z-axis
    gp_Pln plane(planeOrigin, planeNormal);
    std::vector<std::pair<int, double>> angles;
    for (int i = 0; i < indexes.size(); i++)
    {
        gp_Pnt pointToProject1(points[indexes[i]].X(), points[indexes[i]].Y(), points[indexes[i]].Z());
        gp_Vec originToPoint1(planeOrigin, pointToProject1);
        double distance = originToPoint1.Dot(z); // Distance from point to plane along normal
        gp_Pnt projectedPoint1 = pointToProject1.Translated(-distance * z);
        Eigen::Matrix<double, 4, 1> p1(projectedPoint1.X(), projectedPoint1.Y(), projectedPoint1.Z(), 1);
        p1 = trans * p1;
        // Calculate angles using atan2
        double angle = atan2(p1.y(), p1.x());
        angles.push_back(std::make_pair(i, angle));
    }

    std::sort(angles.begin(), angles.end(),
        [](const std::pair<int, double>& a, const std::pair<int, double>& b)
        {
            return a.second < b.second; // Compare by the double value
        });

    std::vector<int> temp;
    for (int i = 0; i < angles.size(); i++)
    {
        temp.push_back(indexes[angles[i].first]);
    }

    indexes.clear();
    for (int i = temp.size() - 1; i >= 0; i--)
        indexes.push_back(temp[i]);
}

bool TopoMesh::checkTheatEqu(std::vector<sv::Point>& points)
{
    for (int j = 0; j < points.size(); j++)
    {
        for (int k = j + 1; k < points.size(); k++)
        {
            if (abs(points[j].theta - points[k].theta) < 1e-5)
                return true;
        }
    }
    return false;
}

void TopoMesh::removeDuplicateVertices()
{
    /// delete duplicate points  in voronoi_vertices
    for (int i = 0; i < voronoi_vertices.size(); i++)
    {
        int left = i;
        int right = voronoi_vertices.size() - 1;
        double precise = 1e-4; // remove duplicate or nearest points, influencing circle shape
        while (left != right)
        {
            if (abs(voronoi_vertices[left].X() - voronoi_vertices[right].X()) < precise && abs(voronoi_vertices[left].Y() - voronoi_vertices[right].Y()) < precise && abs(voronoi_vertices[left].Z() - voronoi_vertices[right].Z()) < precise)
            {
                for (int j = 0; j < voronoi_cells.size(); j++)
                {
                    for (int k = 0; k < voronoi_cells[j].size(); k++)
                    {
                        if (voronoi_cells[j][k] == right)
                        {
                            voronoi_cells[j][k] = left;
                        }
                    }
                }

                for (int j = 0; j < voronoi_cells.size(); j++)
                {
                    for (int k = 0; k < voronoi_cells[j].size(); k++)
                    {
                        if (voronoi_cells[j][k] > right)
                        {
                            voronoi_cells[j][k] = voronoi_cells[j][k] - 1;
                        }
                    }
                }
                std::vector<gp_Pnt>::iterator iter = voronoi_vertices.begin() + right;
                voronoi_vertices.erase(iter);

                // update voronoi_vertices_belong_cell
                for (int j = voronoi_vertices_belong_cell[right].size() - 1; j >= 0; j--)
                {
                    int idx_right = voronoi_vertices_belong_cell[right][j];
                    for (int k = voronoi_vertices_belong_cell[left].size() - 1; k >= 0; k--)
                    {
                        int idx_left = voronoi_vertices_belong_cell[left][k];
                        if (idx_right == idx_left)
                        {
                            std::vector<int>::iterator iter = voronoi_vertices_belong_cell[right].begin() + j;
                            voronoi_vertices_belong_cell[right].erase(iter);
                        }
                    }
                }
                for (int j = 0; j < voronoi_vertices_belong_cell[right].size(); j++)
                {
                    voronoi_vertices_belong_cell[left].push_back(voronoi_vertices_belong_cell[right][j]);
                }

                std::vector<std::vector<int>> temp_belong_cell;
                for (int j = 0; j < voronoi_vertices_belong_cell.size(); j++)
                {
                    if (j != right)
                        temp_belong_cell.push_back(voronoi_vertices_belong_cell[j]);
                }

                voronoi_vertices_belong_cell.clear();
                for (int j = 0; j < temp_belong_cell.size(); j++)
                {
                    voronoi_vertices_belong_cell.push_back(temp_belong_cell[j]);
                }
            }
            right--;
        }
    }

    // delete duplicate index  in cells
    for (int i = 0; i < voronoi_cells.size(); i++)
    {
        int start = voronoi_cells[i].size() - 1;
        while (start > 0)
        {
            int afore = start - 1;
            if (voronoi_cells[i][start] == voronoi_cells[i][afore])
            {
                std::vector<int>::iterator iter = voronoi_cells[i].begin() + start;
                voronoi_cells[i].erase(iter);
            }
            start--;
        }

        start = voronoi_cells[i].size() - 1;
        int afore = 0;
        if (voronoi_cells[i][start] == voronoi_cells[i][afore])
        {
            std::vector<int>::iterator iter = voronoi_cells[i].begin() + start;
            voronoi_cells[i].erase(iter);
        }
    }
}

void TopoMesh::addVtxInEdge_all()
{
    OpenMesh::EPropHandleT<int> touched;
    poly.add_property(touched);

    for (int i = 0; i < voronoi_vertices.size(); i++)
        poly.add_vertex(PolyMesh::Point(voronoi_vertices[i].X(), voronoi_vertices[i].Y(), voronoi_vertices[i].Z()));
    for (int i = 0; i < voronoi_cells.size(); i++)
    {
        std::vector<PolyMesh::VertexHandle> vhandles;
        for (int j = 0; j < voronoi_cells[i].size(); j++)
        {
            PolyMesh::VertexHandle vh = poly.vertex_handle(voronoi_cells[i][j]);
            vhandles.push_back(vh);
        }
        poly.add_face(vhandles);
    }

    for (Mesh::EdgeIter e_it = poly.edges_begin(); e_it != poly.edges_end(); ++e_it)
    {
        Mesh::EdgeHandle eh = *e_it;
        poly.property(touched, eh) = -1; // record either edge touched using flag, touched edge marked with the idx of voronoi vertex
    }
    voronoi_cells.clear();
    for (Mesh::FaceIter f_it = poly.faces_begin(); f_it != poly.faces_end(); ++f_it)
    {
        Mesh::FaceHandle fh = *f_it;

        std::vector<int> Vcell;
        for (PolyMesh::FaceHalfedgeIter fh_it = poly.fh_iter(fh); fh_it.is_valid(); ++fh_it)
        {
            PolyMesh::HalfedgeHandle heh = *fh_it;
            PolyMesh::EdgeHandle eh = poly.edge_handle(heh);
            PolyMesh::VertexHandle vh1 = poly.from_vertex_handle(heh);
            PolyMesh::VertexHandle vh2 = poly.to_vertex_handle(heh);
            if (poly.property(touched, eh) == -1)
            {
                gp_Pnt p1(poly.point(vh1)[0], poly.point(vh1)[1], poly.point(vh1)[2]);
                gp_Pnt p2(poly.point(vh2)[0], poly.point(vh2)[1], poly.point(vh2)[2]);

                PolyMesh::FaceHandle fh1 = poly.face_handle(heh);
                PolyMesh::FaceHandle fh2 = poly.face_handle(poly.opposite_halfedge_handle(heh));

                gp_Vec axisDir_1(latNode.origin, latNode.circles[fh1.idx()].circle_center);
                gp_Vec axisDir_2(latNode.origin, latNode.circles[fh2.idx()].circle_center);
                axisDir_1.Normalize();
                axisDir_2.Normalize();
                gp_Vec vecBetwAxis = axisDir_1 + axisDir_2;
                vecBetwAxis.Normalize();

                gp_Pnt midOfEdge(p1.X() * 0.5 + p2.X() * 0.5, p1.Y() * 0.5 + p2.Y() * 0.5, p1.Z() * 0.5 + p2.Z() * 0.5);
                gp_Vec midDir(latNode.origin, midOfEdge);
                midDir.Normalize();

                gp_Vec endDir_1(latNode.origin, p1);
                gp_Vec endDir_2(latNode.origin, p2);

                Vcell.push_back(vh1.idx());

                if (axisDir_1.Angle(axisDir_2) < 3 && endDir_1.Angle(endDir_2) > 0.5)
                {
                    if ((vecBetwAxis.Angle(endDir_1) < endDir_1.Angle(endDir_2) && vecBetwAxis.Angle(endDir_2) < endDir_1.Angle(endDir_2)))
                    {
                        gp_Pnt addmid(latNode.origin.X() + vecBetwAxis.X() * latNode.sphereRadii,
                            latNode.origin.Y() + vecBetwAxis.Y() * latNode.sphereRadii,
                            latNode.origin.Z() + vecBetwAxis.Z() * latNode.sphereRadii);

                        double a = 0.3, b = 1 - a;
                        gp_Pnt addv1(p1.X() * a + addmid.X() * b, p1.Y() * a + addmid.Y() * b, p1.Z() * a + addmid.Z() * b);
                        gp_Pnt addv2(p2.X() * a + addmid.X() * b, p2.Y() * a + addmid.Y() * b, p2.Z() * a + addmid.Z() * b);
                        gp_Vec vec5(latNode.origin, addv1);
                        gp_Vec vec6(latNode.origin, addv2);
                        vec5.Normalize();
                        vec6.Normalize();

                        if (vec5.Angle(vecBetwAxis) < 0.2 || vec6.Angle(vecBetwAxis) < 0.2)
                        {
                            addmid = gp_Pnt(latNode.origin.X() + midDir.X() * latNode.sphereRadii,
                                latNode.origin.Y() + midDir.Y() * latNode.sphereRadii,
                                latNode.origin.Z() + midDir.Z() * latNode.sphereRadii);
                            a = 0.3;
                            b = 1 - a;
                            addv1 = gp_Pnt(p1.X() * a + addmid.X() * b, p1.Y() * a + addmid.Y() * b, p1.Z() * a + addmid.Z() * b);
                            addv2 = gp_Pnt(p2.X() * a + addmid.X() * b, p2.Y() * a + addmid.Y() * b, p2.Z() * a + addmid.Z() * b);
                            vec5 = gp_Vec(latNode.origin, addv1);
                            vec6 = gp_Vec(latNode.origin, addv2);
                        }

                        addv1.SetX(latNode.origin.X() + vec5.X() * latNode.sphereRadii);
                        addv1.SetY(latNode.origin.Y() + vec5.Y() * latNode.sphereRadii);
                        addv1.SetZ(latNode.origin.Z() + vec5.Z() * latNode.sphereRadii);

                        addv2.SetX(latNode.origin.X() + vec6.X() * latNode.sphereRadii);
                        addv2.SetY(latNode.origin.Y() + vec6.Y() * latNode.sphereRadii);
                        addv2.SetZ(latNode.origin.Z() + vec6.Z() * latNode.sphereRadii);

                        if (axisDir_1.Angle(axisDir_2) < 0.5)
                            vice_intersectPnts.push_back(voronoi_vertices.size());
                        Vcell.push_back(voronoi_vertices.size());
                        poly.property(touched, eh) = voronoi_vertices.size();
                        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(fh.idx());
                        voronoi_vertices.push_back(addv1);

                        if (axisDir_1.Angle(axisDir_2) < 0.5)
                            intersectPnts.push_back(voronoi_vertices.size());
                        Vcell.push_back(voronoi_vertices.size());
                        poly.property(touched, eh) = voronoi_vertices.size();
                        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(fh.idx());
                        voronoi_vertices.push_back(addmid);

                        if (axisDir_1.Angle(axisDir_2) < 0.5)
                            vice_intersectPnts.push_back(voronoi_vertices.size());
                        Vcell.push_back(voronoi_vertices.size());
                        poly.property(touched, eh) = voronoi_vertices.size();
                        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(fh.idx());
                        voronoi_vertices.push_back(addv2);
                    }
                    else
                    {
                        gp_Pnt addmid(latNode.origin.X() + midDir.X() * latNode.sphereRadii,
                            latNode.origin.Y() + midDir.Y() * latNode.sphereRadii,
                            latNode.origin.Z() + midDir.Z() * latNode.sphereRadii);

                        double a = 0.3, b = 1 - a;
                        gp_Pnt addv1(p1.X() * a + addmid.X() * b, p1.Y() * a + addmid.Y() * b, p1.Z() * a + addmid.Z() * b);
                        gp_Pnt addv2(p2.X() * a + addmid.X() * b, p2.Y() * a + addmid.Y() * b, p2.Z() * a + addmid.Z() * b);
                        gp_Vec vec5(latNode.origin, addv1);
                        gp_Vec vec6(latNode.origin, addv2);
                        vec5.Normalize();
                        vec6.Normalize();

                        addv1.SetX(latNode.origin.X() + vec5.X() * latNode.sphereRadii);
                        addv1.SetY(latNode.origin.Y() + vec5.Y() * latNode.sphereRadii);
                        addv1.SetZ(latNode.origin.Z() + vec5.Z() * latNode.sphereRadii);

                        addv2.SetX(latNode.origin.X() + vec6.X() * latNode.sphereRadii);
                        addv2.SetY(latNode.origin.Y() + vec6.Y() * latNode.sphereRadii);
                        addv2.SetZ(latNode.origin.Z() + vec6.Z() * latNode.sphereRadii);

                        if (axisDir_1.Angle(axisDir_2) < 0.5)
                            vice_intersectPnts.push_back(voronoi_vertices.size());
                        Vcell.push_back(voronoi_vertices.size());
                        poly.property(touched, eh) = voronoi_vertices.size();
                        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(fh.idx());
                        voronoi_vertices.push_back(addv1);

                        if (axisDir_1.Angle(axisDir_2) < 0.5)
                            intersectPnts.push_back(voronoi_vertices.size());
                        Vcell.push_back(voronoi_vertices.size());
                        poly.property(touched, eh) = voronoi_vertices.size();
                        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(fh.idx());
                        voronoi_vertices.push_back(addmid);

                        if (axisDir_1.Angle(axisDir_2) < 0.5)
                            vice_intersectPnts.push_back(voronoi_vertices.size());
                        Vcell.push_back(voronoi_vertices.size());
                        poly.property(touched, eh) = voronoi_vertices.size();
                        voronoi_vertices_belong_cell[voronoi_vertices.size()].push_back(fh.idx());
                        voronoi_vertices.push_back(addv2);
                    }
                }
            }
            else
            {
                Vcell.push_back(vh1.idx());
                Vcell.push_back(poly.property(touched, eh));
                Vcell.push_back(poly.property(touched, eh) - 1);
                Vcell.push_back(poly.property(touched, eh) - 2);
                voronoi_vertices_belong_cell[poly.property(touched, eh)].push_back(fh.idx());
                voronoi_vertices_belong_cell[poly.property(touched, eh) - 1].push_back(fh.idx());
                voronoi_vertices_belong_cell[poly.property(touched, eh) - 2].push_back(fh.idx());
            }
        }
        voronoi_cells.push_back(Vcell);
    }
}

void TopoMesh::proVronoiVtxToNode()
{
    for (int i = 0; i < voronoi_vertices.size(); i++)
    {

        gp_Pnt origin = latNode.origin;
        gp_Pnt pnt = latNode.circles[voronoi_vertices_belong_cell[i][0]].circle_center;
        gp_Pnt point = voronoi_vertices[i];
        gp_Ax3 axis(origin, gp_Dir(gp_Vec(origin, pnt)));
        gp_Cylinder cylinder(axis, latNode.circles[voronoi_vertices_belong_cell[i][0]].circle_radius);
        IntAna_Quadric quadCylinder(cylinder);
        IntAna_IntConicQuad inta(gp_Lin(origin, gp_Dir(gp_Vec(origin, point))), quadCylinder);
        gp_Pnt p1 = inta.Point(1);
        gp_Pnt p2 = inta.Point(2);
        gp_Pnt projection = p1.Distance(point) < p2.Distance(point) ? p1 : p2;
        voronoi_vertices[i] = projection;
    }
}

void TopoMesh::meshFromVoronoiDiagram_irregularHandle()
{
    std::vector<Mesh::VertexHandle> vhandles;
    for (int i = 0; i < voronoi_vertices.size(); i++)
    {
        Mesh::VertexHandle vh = initialMesh.mesh.add_vertex(Mesh::Point(voronoi_vertices[i].X(), voronoi_vertices[i].Y(), voronoi_vertices[i].Z()));
        vhandles.push_back(vh);
        VProp prop;
        prop.belongStrut = -1;
        prop.hasNormal = false;
        prop.isHandled = false;
        initialMesh.vprop.push_back(prop);
    }

    for (int i = 0; i < intersectPnts.size(); i++)
    {
        initialMesh.vprop[intersectPnts[i]].belongStrut = -2;
    }

    for (int i = 0; i < vice_intersectPnts.size(); i++)
    {
        initialMesh.vprop[vice_intersectPnts[i]].belongStrut = -3;
    }

    for (int i = 0; i < voronoi_cells.size(); i++)
    {
        std::vector<int> cirLayer;
        cirDivideProjection(i, initialMesh.mesh, vhandles, cirLayer);

        for (int k = 0; k < cirLayer.size(); k++)
        {
            Mesh::Point pnt = initialMesh.mesh.point(vhandles[cirLayer[k]]);
            latNode.circles[i].cirPoints.push_back(gp_Pnt(pnt[0], pnt[1], pnt[2]));
        }

        int layerSize = 3; // 3
        std::vector<std::vector<int>> layers(layerSize + 2);

        layers[0] = voronoi_cells[i];
        for (int k = 0; k < layerSize; k++)
        {
            double a = 0.6; // 0.4
            double alpha = a + (1.0 - a) / (layerSize + 1.0) * (k + 1);
            for (int j = 0; j < voronoi_cells[i].size(); j++)
            {
                Mesh::Point pnt = initialMesh.mesh.point(vhandles[cirLayer[j]]);
                Mesh::Point pnt1 = pnt * alpha + initialMesh.mesh.point(vhandles[voronoi_cells[i][j]]) * (1 - alpha);
                // Mesh::Point pnt1 = pnt * (1 - alpha * (k + 1)) + initialMesh.mesh.point(vhandles[voronoi_cells[i][j]]) * alpha * (k + 1);
                gp_Pnt point(pnt1[0], pnt1[1], pnt1[2]);
                gp_Pnt pntOnCylin = PointToSegDist(point, latNode.origin, latNode.circles[i].circle_center);
                gp_Vec vec(pntOnCylin, point);
                vec.Normalize();
                double radii = latNode.circles[i].circle_radius;
                pnt1 = Mesh::Point(pntOnCylin.X() + vec.X() * radii, pntOnCylin.Y() + vec.Y() * radii, pntOnCylin.Z() + vec.Z() * radii);
                layers[k + 1].push_back(vhandles.size());
                Mesh::VertexHandle vh = initialMesh.mesh.add_vertex(pnt1);
                if (k >= layerSize - 2) //-2
                {
                    VProp prop;
                    prop.belongStrut = i;
                    if (k >= layerSize - 1) //-1
                    {
                        prop.hasNormal = true;
                        prop.normal = vec;
                        prop.normal.Normalize();
                    }
                    prop.isHandled = true;

                    if (k == layerSize - 2 && (initialMesh.vprop[voronoi_cells[i][j]].belongStrut == -2 || initialMesh.vprop[voronoi_cells[i][j]].belongStrut == -3))
                        prop.isHandled = false;

                    prop.normal = vec; // for G1 boundary condition
                    prop.normal.Normalize();

                    initialMesh.vprop.push_back(prop);
                }
                else
                {
                    VProp prop;
                    initialMesh.vprop.push_back(prop);
                }
                vhandles.push_back(vh);
            }
        }
        layers[layers.size() - 1] = cirLayer;

        // for (int j = 2; j < layers.size(); j++)
        // {
        //     for (int k = 0; k < layers[j].size(); k++)
        //     {
        //         Mesh::Point p = initialMesh.mesh.point(vhandles[layers[j][k]]);
        //         std::cout << p[0] << " " << p[1] << " " << p[2] << std::endl;
        //     }
        // }

        int poly1_id = -1, poly2_id = -1, cir1_id = -1, cir2_id = -1;
        for (int j = 0; j < voronoi_cells[i].size(); j++)
        {
            std::vector<Mesh::VertexHandle> face_vhandles;
            for (int mk = 0; mk < layers.size() - 1; mk++)
            {
                poly1_id = layers[mk][j];
                poly2_id = layers[mk][(j + 1) % voronoi_cells[i].size()];
                cir1_id = layers[mk + 1][j];
                cir2_id = layers[mk + 1][(j + 1) % voronoi_cells[i].size()];

                face_vhandles.clear();
                face_vhandles.push_back(vhandles[poly1_id]);
                face_vhandles.push_back(vhandles[cir1_id]);
                face_vhandles.push_back(vhandles[poly2_id]);
                initialMesh.mesh.add_face(face_vhandles);

                face_vhandles.clear();
                face_vhandles.push_back(vhandles[poly2_id]);
                face_vhandles.push_back(vhandles[cir1_id]);
                face_vhandles.push_back(vhandles[cir2_id]);
                initialMesh.mesh.add_face(face_vhandles);
            }
        }
    }
}

void TopoMesh::cirDivideProjection(int circle_id, Mesh& mesh, std::vector<Mesh::VertexHandle>& vhandles, std::vector<int>& points)
{
    VProp prop;
    gp_Pnt cirCenter = latNode.circles[circle_id].circle_center;
    double radii = latNode.circles[circle_id].circle_radius;
    for (int j = 0; j < voronoi_cells[circle_id].size(); j++)
    {
        gp_Pnt pvoro = voronoi_vertices[voronoi_cells[circle_id][j]];
        gp_Vec dir(latNode.origin, cirCenter);
        dir.Normalize();
        gp_Pln pln(cirCenter, dir);
        double distance = pln.Distance(pvoro);

        Mesh::Point pnt(pvoro.X() + dir.X() * distance, pvoro.Y() + dir.Y() * distance, pvoro.Z() + dir.Z() * distance);
        Mesh::VertexHandle vh = mesh.add_vertex(pnt);
        vhandles.push_back(vh);
        prop.belongStrut = circle_id;
        prop.hasNormal = true;
        prop.normal = gp_Vec(cirCenter, gp_Pnt(pnt[0], pnt[1], pnt[2]));
        prop.normal.Normalize();
        prop.isHandled = true;
        prop.isBoundary = true;
        initialMesh.vprop.push_back(prop);
        Mesh::Point pntOnCir(cirCenter.X() + prop.normal.X() * radii, cirCenter.Y() + prop.normal.Y() * radii, cirCenter.Z() + prop.normal.Z() * radii);
        mesh.set_point(vh, pntOnCir);
        points.push_back(vh.idx());
    }
}

gp_Pnt TopoMesh::PointToSegDist(gp_Pnt& pnt, const gp_Pnt& edge_v1, const gp_Pnt& edge_v2)
{
    TopoDS_Vertex vertex = BRepBuilderAPI_MakeVertex(pnt);
    Handle(Geom_TrimmedCurve) aSegment1 = GC_MakeSegment(edge_v1, edge_v2);
    TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(aSegment1);

    BRepExtrema_DistShapeShape DistVE(vertex, edge);
    int num = DistVE.NbSolution();

    return DistVE.PointOnShape2(1);
}

TopoMesh::~TopoMesh()
{
}