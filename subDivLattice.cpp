
#include <string>
#include <iostream>
#include "nodeManager.h"
#include "TopoMesh.h"
#include "OptiMesh.h"
#include "SubDivMesh.h"

void writeLattice(std::vector<Mesh>& meshes, std::string& path);

int main()
{
    std::cout << "input the path of .node file:" << std::endl;
	std::string nodePath="data/cube.node";
    std::cin >> nodePath;

    std::cout << "input the path of .edge file:" << std::endl;
	std::string edgePath="data/cube.edge";
    std::cin >> edgePath;

    std::string cylinderSTLpath = "../../subDivLattice/results/cube_cylinder.stl";
    std::string latticeSTLpath = "../../subDivlattice/results/cube_nodes.stl";

    NodeManager nodeMa;
    nodeMa.nodeReading(nodePath, edgePath);

    // checkEdgeOverlap(nodeMa);

    int subdivIter = 1;
    int lapIter = 2;
    int nodeSize = nodeMa.node_array.size();

    std::vector<Mesh> meshes;
    int progress = 0;
    std::cout << "the num of all nodes: " << nodeMa.node_array.size() << std::endl;

    for (int i = 0; i < nodeSize; i++)
    {
        if (nodeMa.node_array[i].circles.size() < 3)
        {
            std::cout << "invalid " << i << "th graph node: the num of struts less than 3." << std::endl;
            assert(nodeMa.node_array[i].circles.size() >= 3);
        }

        if (nodeMa.checkCoplanar(nodeMa.node_array[i]))
        {
            std::cout << "invalid " << i << "th graph node: the struts are coplanar." << std::endl;
            assert(!nodeMa.checkCoplanar(nodeMa.node_array[i]));
        }

        TopoMesh topoMesh(nodeMa.node_array[i]);
        topoMesh.buildInitialMesh();

        OptiMesh optiMesh(topoMesh.initialMesh);
        optiMesh.laplacianFairing(lapIter);

        SubDivMesh subDiv(nodeMa.node_array[i], topoMesh.initialMesh);
        subDiv.calculate_vertex_normals(topoMesh.initialMesh, 0);

        for (int iter = 0; iter < subdivIter; iter++)
            subDiv.PNLoopSubDiv();

        if ((int)((double)(i + 1) / (double)nodeMa.node_array.size() * 10) == progress)
        {
            std::cout << "progress: " << progress * 10 << "%" << std::endl;
            progress++;
        }
         meshes.push_back(topoMesh.initialMesh.mesh);
    }

     // exporting lattice stl
    nodeMa.triangulateCylinder_waterTight(cylinderSTLpath, subdivIter);
    writeLattice(meshes, latticeSTLpath);
}

void writeLattice(std::vector<Mesh>& meshes, std::string& path)
{
    std::cout << ">>>>>>Exporting lattice STL..." << std::endl;
    std::ofstream file(path);
    if (!file)
        std::cout << ">>>>>>ERROR::lattice .stl file open failure." << std::endl;
    assert(file);
    file << "solid WRAP" << std::endl;

    int val = meshes.size();
    for (int j = 0; j < val; j++)
    {
        for (Mesh::FaceIter f_it = meshes[j].faces_begin(); f_it != meshes[j].faces_end(); ++f_it)
        {
            Mesh::FaceHandle fh = *f_it;
            std::vector<gp_Pnt> points;
            for (Mesh::FaceVertexIter fv_it = meshes[j].fv_iter(fh); fv_it.is_valid(); ++fv_it)
            {
                Mesh::Point point = meshes[j].point(*fv_it); // Get the point (vertex coordinates)
                points.push_back(gp_Pnt(point[0], point[1], point[2]));
            }
            gp_Vec dir1(points[2], points[0]);
            gp_Vec dir2(points[1], points[0]);
            gp_Vec dirZ(0, 0, 1);
            if (dir1.Angle(dir2) > 1e-6)
            {
                dirZ = dir2.Crossed(dir1);
                dirZ.Normalize();
            }
            else
            {
                std::cout << "there is degenerated triangles in lattice stl. " << std::endl;
            }

            file << "facet normal " << dirZ.X() << " " << dirZ.Y() << " " << dirZ.Z() << "\n";
            file << "outer loop"
                << "\n";
            file << "vertex " << points[0].X() << " " << points[0].Y() << " " << points[0].Z() << std::endl
                << "vertex " << points[1].X() << " " << points[1].Y() << " " << points[1].Z() << std::endl
                << "vertex " << points[2].X() << " " << points[2].Y() << " " << points[2].Z() << "\n";
            file << "endloop"
                << "\n";
            file << "endfacet"
                << "\n";
        }
    }

    file << "endsolid WRAP" << std::endl;
    file.close();
    std::cout << ">>>>>>Lattice STL has been exported!" << std::endl;
}