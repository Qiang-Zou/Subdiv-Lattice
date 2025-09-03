#pragma once
#pragma once
#include "VisualTool.h"
#include <map>

struct circle
{
	gp_Pnt circle_center;
	double circle_radius;
	std::vector<gp_Pnt> cirPoints;
};

struct node
{
	gp_Pnt origin;
	std::vector<circle> circles;
	std::vector<int> edge_ids;
	double sphereRadii;
};

struct node_edge
{
	int node_idx_1 = -1;
	int node_idx_2 = -1;
	int node_circle_idx_1 = -1;
	int node_circle_idx_2 = -1;
	double radius;
};

class NodeManager
{
public:
	NodeManager();
	~NodeManager();
	void nodeReading(std::string& nodepath, std::string& edgepath);
	void nodepairCutting(std::vector<node>& nodeArray);
	void readPoint(std::vector<node>& nodeArray, std::string& nodepath);
	void readEdge(std::vector<node>& nodeArray, std::string& edgepath);
	void triangulateCylinder_waterTight(std::string& path, int iteration);
	void writeTriangle(std::ofstream& file, const gp_Pnt& p1, const gp_Pnt& p2, const gp_Pnt& p3);
	bool checkCoplanar(node& _node);
	void findBestIndexMatch(const std::vector<gp_Pnt>& smallerCircle, const std::vector<gp_Pnt>& largerCircle, std::vector<int>& mapping);

public:
	std::vector<node> node_array;
	std::vector<node_edge> node_edges;

private:
};
