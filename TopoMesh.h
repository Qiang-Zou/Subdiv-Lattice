#pragma once

#include "VisualTool.h"
#include "svFortune/svVoronoiCore.h"

struct node;

enum VoronoiType
{
	Fortune,
	Hull
};

class TopoMesh
{
public:
	TopoMesh(node& _node);
	~TopoMesh();
	void FortuneVoronoi();
	bool checkTheatEqu(std::vector<sv::Point>& points);
	void reorderPointsCCW(const gp_Pnt& circleCenter, std::vector<int>& indexes, const std::vector<gp_Pnt>& points);
	void removeDuplicateVertices();
	void buildInitialMesh();
	void addVtxInEdge_all();
	void proVronoiVtxToNode();
	void meshFromVoronoiDiagram_irregularHandle();
	void cirDivideProjection(int circle_id, Mesh& mesh, std::vector<Mesh::VertexHandle>& vhandles, std::vector<int>& points);
	gp_Pnt PointToSegDist(gp_Pnt& pnt, const gp_Pnt& edge_v1, const gp_Pnt& edge_v2);

public:
	MyMesh initialMesh;
	node& latNode;
	PolyMesh poly;
	VoronoiType voronoiType;
	std::vector<gp_Pnt> voronoi_vertices;
	std::vector<std::vector<int>> voronoi_vertices_belong_cell;
	std::vector<std::vector<int>> voronoi_cells;
	std::vector<int> intersectPnts;
	std::vector<int> vice_intersectPnts;
};