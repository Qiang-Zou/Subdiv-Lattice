#include "subDivMesh.h"

SubDivMesh::SubDivMesh(node& _node, MyMesh& _myMesh) : latNode(_node), myMesh(_myMesh)
{
}

SubDivMesh::~SubDivMesh()
{
}

void SubDivMesh::PNLoopSubDiv()
{
    MyMesh tempMesh;
    OpenMesh::EPropHandleT<Mesh::VertexHandle> e_vertex;
    myMesh.mesh.add_property(e_vertex);
    // calculate_vertex_normals(myMesh, 0);
    // update oldvertices
    for (Mesh::VertexIter viter = myMesh.mesh.vertices_begin(); viter != myMesh.mesh.vertices_end(); ++viter)
    {
        Mesh::VertexHandle v = *viter;
        // Crease
        if (myMesh.vprop[v.idx()].isBoundary)
        {
            tempMesh.mesh.add_vertex(myMesh.mesh.point(v));
            tempMesh.vprop.push_back(myMesh.vprop[v.idx()]);
        }
        else
        {
            // examine all neighboring vertices
            std::vector<Mesh::Point> plist;
            std::vector<gp_Vec> nlist;
            for (Mesh::VertexVertexIter vv_it = myMesh.mesh.vv_iter(v); vv_it.is_valid(); ++vv_it)
            {
                Mesh::VertexHandle vh = *vv_it;
                plist.push_back(myMesh.mesh.point(vh));
                nlist.push_back(myMesh.vprop[v.idx()].normal);
            }

            int n = plist.size();
            float alpha = calculateAlpha(n);
            Mesh::Point temp = { 0.0f, 0.0f, 0.0f };
            Mesh::Point ntemp = { 0.0f, 0.0f, 0.0f };
            for (int i = 0; i < n; i++)
            {
                temp += plist.back();
                plist.pop_back();
                Mesh::Point pnt(nlist.back().X(), nlist.back().Y(), nlist.back().Z());
                ntemp += pnt;
                nlist.pop_back();
            }
            Mesh::Point newPoint = myMesh.mesh.point(v) * (1 - n * alpha) + temp * alpha;
            Mesh::Point VN = Mesh::Point(myMesh.vprop[v.idx()].normal.X(), myMesh.vprop[v.idx()].normal.Y(), myMesh.vprop[v.idx()].normal.Z()) * (1.0 - n * alpha) + ntemp * alpha;
            gp_Vec newNormal(VN[0], VN[1], VN[2]);
            newNormal.Normalize();
            std::vector<double> hlist;
            for (Mesh::VertexVertexIter vv_it = myMesh.mesh.vv_iter(v); vv_it.is_valid(); ++vv_it)
            {
                Mesh::VertexHandle vh = *vv_it;
                gp_Vec ni = newNormal;
                gp_Vec nj = myMesh.vprop[vh.idx()].normal;
                gp_Pnt pi(newPoint[0], newPoint[1], newPoint[2]);
                gp_Pnt pj(myMesh.mesh.point(vh)[0], myMesh.mesh.point(vh)[1], myMesh.mesh.point(vh)[2]);
                double val = computeHij(ni, nj, pi, pj);
                hlist.push_back(val);
            }

            double htemp = 0;
            for (int i = 0; i < n; i++)
            {
                htemp += hlist.back();
                hlist.pop_back();
            }
            gp_Pnt pi(newPoint[0], newPoint[1], newPoint[2]);
            gp_Pnt pj(myMesh.mesh.point(v)[0], myMesh.mesh.point(v)[1], myMesh.mesh.point(v)[2]);
            double aveHij = computeHij(newNormal, myMesh.vprop[v.idx()].normal, pi, pj) * (1 - n * alpha) + htemp * alpha;
            gp_Vec tempPnt = newNormal * aveHij;
            newPoint = newPoint + Mesh::Point(tempPnt.X(), tempPnt.Y(), tempPnt.Z());
            tempMesh.mesh.add_vertex(newPoint);
            myMesh.vprop[v.idx()].normal = newNormal;
            tempMesh.vprop.push_back(myMesh.vprop[v.idx()]);
        }
    }

    // scan all edges and create vertex on each edge
    std::vector<Mesh::VertexHandle> e_vertex_vh;
    for (Mesh::EdgeIter eiter = myMesh.mesh.edges_begin(); eiter != myMesh.mesh.edges_end(); ++eiter)
    {
        Mesh::EdgeHandle e = *eiter;
        Mesh::HalfedgeHandle hf = myMesh.mesh.halfedge_handle(e, 0);
        Mesh::VertexHandle ev1 = myMesh.mesh.from_vertex_handle(hf);
        Mesh::VertexHandle ev2 = myMesh.mesh.to_vertex_handle(hf);
        // Crease
        if (myMesh.vprop[ev1.idx()].isBoundary && myMesh.vprop[ev2.idx()].isBoundary)
        {
            gp_Pnt origin = latNode.circles[myMesh.vprop[ev1.idx()].belongStrut].circle_center;
            double scale = latNode.circles[myMesh.vprop[ev1.idx()].belongStrut].circle_radius;
            gp_Pnt p1(myMesh.mesh.point(ev1)[0], myMesh.mesh.point(ev1)[1], myMesh.mesh.point(ev1)[2]);
            gp_Pnt p2(myMesh.mesh.point(ev2)[0], myMesh.mesh.point(ev2)[1], myMesh.mesh.point(ev2)[2]);

            gp_Vec v1(origin, p1);
            gp_Vec v2(origin, p2);
            v1.Normalize();
            v2.Normalize();
            gp_Vec vec = v1 + v2;
            vec.Normalize();
            Mesh::VertexHandle vh = tempMesh.mesh.add_vertex(Mesh::Point(origin.X() + vec.X() * scale, origin.Y() + vec.Y() * scale, origin.Z() + vec.Z() * scale));
            myMesh.mesh.property(e_vertex, e) = vh;
            VProp prop;
            prop.belongStrut = myMesh.vprop[ev1.idx()].belongStrut > myMesh.vprop[ev2.idx()].belongStrut ? myMesh.vprop[ev1.idx()].belongStrut : myMesh.vprop[ev2.idx()].belongStrut;
            prop.isBoundary = true;
            prop.hasNormal = true;
            prop.normal = vec;
            tempMesh.vprop.push_back(prop);
        }
        else
        {
            Mesh::Point newPoint = (myMesh.mesh.point(ev1) + myMesh.mesh.point(ev2)) * 0.375;
            Mesh::HalfedgeHandle vt1 = myMesh.mesh.next_halfedge_handle(myMesh.mesh.halfedge_handle(e, 0));
            Mesh::HalfedgeHandle vt2 = myMesh.mesh.next_halfedge_handle(myMesh.mesh.halfedge_handle(e, 1));
            Mesh::VertexHandle pnt1 = myMesh.mesh.to_vertex_handle(vt1);
            Mesh::VertexHandle pnt2 = myMesh.mesh.to_vertex_handle(vt2);
            newPoint = newPoint + (myMesh.mesh.point(pnt1) + myMesh.mesh.point(pnt2)) * 0.125;

            gp_Vec nev1, nev2, nvt1, nvt2;

            nev1 = myMesh.vprop[ev1.idx()].normal;
            nev2 = myMesh.vprop[ev2.idx()].normal;
            nvt1 = myMesh.vprop[pnt1.idx()].normal;
            nvt2 = myMesh.vprop[pnt2.idx()].normal;

            gp_Vec newNormal = (nev1 + nev2) * 0.375f;
            newNormal = newNormal + (nvt1 + nvt2) * 0.125f;
            newNormal.Normalize();
            gp_Vec ni = newNormal;
            gp_Pnt pi(newPoint[0], newPoint[1], newPoint[2]);
            gp_Pnt pj1 = gp_Pnt(myMesh.mesh.point(ev1)[0], myMesh.mesh.point(ev1)[1], myMesh.mesh.point(ev1)[2]);
            gp_Pnt pj2 = gp_Pnt(myMesh.mesh.point(ev2)[0], myMesh.mesh.point(ev2)[1], myMesh.mesh.point(ev2)[2]);
            gp_Pnt pj3 = gp_Pnt(myMesh.mesh.point(pnt1)[0], myMesh.mesh.point(pnt1)[1], myMesh.mesh.point(pnt1)[2]);
            gp_Pnt pj4 = gp_Pnt(myMesh.mesh.point(pnt2)[0], myMesh.mesh.point(pnt2)[1], myMesh.mesh.point(pnt2)[2]);
            double hij_1 = computeHij(ni, nev1, pi, pj1);
            double hij_2 = computeHij(ni, nev2, pi, pj2);
            double hij_3 = computeHij(ni, nvt1, pi, pj3);
            double hij_4 = computeHij(ni, nvt2, pi, pj4);
            double aveHij = (hij_1 + hij_2) * 0.375f + (hij_3 + hij_4) * 0.125f;

            gp_Vec tempPnt = newNormal * aveHij;
            newPoint = newPoint + Mesh::Point(tempPnt.X(), tempPnt.Y(), tempPnt.Z());

            Mesh::VertexHandle vh = tempMesh.mesh.add_vertex(newPoint);
            myMesh.mesh.property(e_vertex, e) = vh;
            VProp prop;
            prop.belongStrut = myMesh.vprop[ev1.idx()].belongStrut > myMesh.vprop[ev2.idx()].belongStrut ? myMesh.vprop[ev1.idx()].belongStrut : myMesh.vprop[ev2.idx()].belongStrut;
            prop.normal = newNormal;
            tempMesh.vprop.push_back(prop);
        }
    }

    ////create faces
    for (Mesh::FaceIter fiter = myMesh.mesh.faces_begin(); fiter != myMesh.mesh.faces_end(); ++fiter)
    {
        Mesh::FaceHandle f = *fiter;
        Mesh::HalfedgeHandle fhe[3];
        fhe[0] = myMesh.mesh.halfedge_handle(f);
        fhe[1] = myMesh.mesh.next_halfedge_handle(fhe[0]);
        fhe[2] = myMesh.mesh.next_halfedge_handle(fhe[1]);

        int boundary_edge_index = -1;

        if (myMesh.vprop[myMesh.mesh.from_vertex_handle(fhe[0]).idx()].isBoundary && myMesh.vprop[myMesh.mesh.to_vertex_handle(fhe[0]).idx()].isBoundary)
            boundary_edge_index = 0;
        if (myMesh.vprop[myMesh.mesh.from_vertex_handle(fhe[1]).idx()].isBoundary && myMesh.vprop[myMesh.mesh.to_vertex_handle(fhe[1]).idx()].isBoundary)
            boundary_edge_index = 1;
        if (myMesh.vprop[myMesh.mesh.from_vertex_handle(fhe[2]).idx()].isBoundary && myMesh.vprop[myMesh.mesh.to_vertex_handle(fhe[2]).idx()].isBoundary)
            boundary_edge_index = 2;

        // Vertex * v[3];
        std::vector<Mesh::VertexHandle> v;
        // create the central small triangle
        for (int i = 0; i < 3; i++)
        {
            Mesh::EdgeHandle eh = myMesh.mesh.edge_handle(fhe[i]);
            v.push_back(myMesh.mesh.property(e_vertex, eh));
        }
        tempMesh.mesh.add_face(v);

        // create small triangles in three corners
        for (int i = 0; i < 3; i++)
        {
            v.clear();
            v.push_back(myMesh.mesh.property(e_vertex, myMesh.mesh.edge_handle(fhe[i])));
            v.push_back(myMesh.mesh.to_vertex_handle(fhe[i]));
            v.push_back(myMesh.mesh.property(e_vertex, myMesh.mesh.edge_handle(myMesh.mesh.next_halfedge_handle(fhe[i]))));
            tempMesh.mesh.add_face(v);
        }
    }
    myMesh = tempMesh;
}

float SubDivMesh::calculateAlpha(int n)
{
    float alpha;
    if (n > 3)
    {
        float center = (0.375f + (0.25f * cos(6.2831853f / (float)n))); // 2.0f * 3.1415926f
        alpha = (0.625f - (center * center)) / (float)n;
    }
    else
    {
        alpha = 0.1875f; // 3.0f / 16.0f;
    }
    return alpha;
}

double SubDivMesh::computeHij(gp_Vec& ni, gp_Vec& nj, gp_Pnt& pi, gp_Pnt& pj)
{
    gp_Vec vec1 = ni + nj;
    gp_Vec vec2(pi, pj);
    if (vec1.Magnitude() == 0 || ni.Magnitude() == 0)
        return 0;
    else
        return vec1.Dot(vec2) / (ni.Dot(vec1));
}

void SubDivMesh::calculate_vertex_normals(MyMesh& myMesh, int type)
{
    // Initialize vertex normals to zero
    for (auto v : myMesh.mesh.vertices())
    {
        if ((!myMesh.vprop[v.idx()].hasNormal) && type == 0)
        {
            gp_Vec vertex_normal(0, 0, 0);
            for (auto f : myMesh.mesh.vf_range(v))
            {
                std::vector<gp_Pnt> pnts;
                for (auto vt : myMesh.mesh.fv_range(f))
                {
                    gp_Pnt point(myMesh.mesh.point(vt)[0], myMesh.mesh.point(vt)[1], myMesh.mesh.point(vt)[2]);
                    pnts.push_back(point);
                }
                gp_Vec vec1(pnts[0], pnts[1]);
                gp_Vec vec2(pnts[0], pnts[2]);

                gp_Vec vec = vec1.Crossed(vec2);
                if (vec1.Angle(vec2) < 1e-6)
                {
                    std::cout << "ERROR::exisiting degenerate triangles...the mini angle in triangle: " << vec1.Angle(vec2) << std::endl;
                    assert(vec1.Angle(vec2) > 1e-6);
                }
                vec.Normalize();
                vertex_normal = vertex_normal + vec;
            }

            vertex_normal.Normalize();
            myMesh.vprop[v.idx()].normal = vertex_normal;
            myMesh.vprop[v.idx()].hasNormal = true;
        }

        if (type == 1)
        {
            gp_Vec vertex_normal(0, 0, 0);
            for (auto f : myMesh.mesh.vf_range(v))
            {
                std::vector<gp_Pnt> pnts;
                for (auto vt : myMesh.mesh.fv_range(f))
                {
                    gp_Pnt point(myMesh.mesh.point(vt)[0], myMesh.mesh.point(vt)[1], myMesh.mesh.point(vt)[2]);
                    pnts.push_back(point);
                }
                gp_Vec vec1(pnts[0], pnts[1]);
                gp_Vec vec2(pnts[0], pnts[2]);
                gp_Vec vec = vec1.Crossed(vec2);
                vec.Normalize();
                vertex_normal = vertex_normal + vec;
            }

            vertex_normal.Normalize();
            myMesh.vprop[v.idx()].normal = vertex_normal;
            myMesh.vprop[v.idx()].hasNormal = true;
        }
    }
}