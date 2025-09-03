#pragma once

#include "VisualTool.h"
#include "nodeManager.h"

class SubDivMesh
{
private:
public:
    SubDivMesh(node& _node, MyMesh& _myMesh);
    ~SubDivMesh();
    void PNLoopSubDiv();
    float calculateAlpha(int n);
    double computeHij(gp_Vec& ni, gp_Vec& nj, gp_Pnt& pi, gp_Pnt& pj);
    void calculate_vertex_normals(MyMesh& myMesh, int type);

private:
    node& latNode;
    MyMesh& myMesh;
};