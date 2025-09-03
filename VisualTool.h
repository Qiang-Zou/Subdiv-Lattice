#pragma once
#define _USE_MATH_DEFINES
#include <OpenMesh/Core/Utils/Property.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/IO/reader/STLReader.hh>
#include <OpenMesh/Core/IO/MeshIO.hh>
#undef _USE_MATH_DEFINES
typedef OpenMesh::TriMesh_ArrayKernelT<> Mesh;
typedef OpenMesh::PolyMesh_ArrayKernelT<> PolyMesh;

#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <vector>
#include <iostream>
#include <assert.h>
#include <tuple>

#include <BRep_Builder.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <GC_MakeSegment.hxx>
#include <GC_MakeCircle.hxx>
#include <TopoDS_Edge.hxx>
#include <gp_Ax2.hxx>
#include <gp_Cylinder.hxx>
#include <IntAna_Quadric.hxx>
#include <IntAna_IntConicQuad.hxx>
#include <Geom_Circle.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_UniformAbscissa.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepExtrema_DistShapeShape.hxx>

#include <TopoDS_Face.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Plane.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>

#include <IGESControl_Controller.hxx>
#include <IGESControl_Writer.hxx>
#include <STEPControl_Controller.hxx>
#include <STEPControl_Writer.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax2.hxx>
#include <gp_Pln.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Compound.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepAlgoAPI_Fuse.hxx>

// spherical voronoi
#include <TopoDS_Vertex.hxx>
#include <queue>
#include <cmath>
#include <map>
#include <algorithm>

#include <Eigen/Dense>
#include <Eigen/Sparse>

using SparseMatrix = Eigen::SparseMatrix<double>;
using DenseMatrix = Eigen::MatrixXd;
using Triplet = Eigen::Triplet<double>;
using DiagonalMatrix = Eigen::DiagonalMatrix<double, Eigen::Dynamic>;

struct VProp
{
	gp_Vec normal = gp_Vec(0, 0, 0);
	int belongStrut = -2;
	bool isHandled = false;
	bool hasNormal = false;
	bool isBoundary = false;
	bool vlocked = false;

	gp_Pnt center;
	double radius;
};

struct MyMesh
{
	Mesh mesh;
	std::vector<VProp> vprop;
};

class VisualTool
{
public:
	VisualTool() {};
	~VisualTool() {};

private:
};
