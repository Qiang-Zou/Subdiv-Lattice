#include "nodeManager.h"

NodeManager::NodeManager()
{
}

NodeManager::~NodeManager()
{
}

void NodeManager::nodeReading(std::string& nodepath, std::string& edgepath)
{
    readPoint(node_array, nodepath);
    readEdge(node_array, edgepath);
    nodepairCutting(node_array);
}

void NodeManager::readPoint(std::vector<node>& nodeArray, std::string& nodepath)
{
    std::ifstream file(nodepath);
    if (!file)
        std::cout << ">>>>>>>ERROR::.node file load failure" << std::endl;
    assert(file);
    std::string s1;
    double x, y, z;
    int a;
    node tempNode;
    std::getline(file, s1);
    while (std::getline(file, s1))
    {
        std::stringstream ss;
        if (s1.compare(0, 1, "#") == 0)
        {
            continue;
        }
        else
        {
            ss << s1;
            ss >> a;
            ss >> x;
            ss >> y;
            ss >> z;
        }
        tempNode.origin = gp_Pnt(x, y, z);
        nodeArray.push_back(tempNode);
    }
}

void NodeManager::readEdge(std::vector<node>& nodeArray, std::string& edgepath)
{
    std::ifstream file(edgepath);
    if (!file)
        std::cout << ">>>>>>>ERROR::.edge file load failure" << std::endl;
    assert(file);
    std::string s1;
    int a1, a2, a3;
    double minLength = 1000;
    std::getline(file, s1);

    while (std::getline(file, s1))
    {
        std::stringstream ss;
        if (s1.compare(0, 1, "#") == 0)
        {
            continue;
        }
        else
        {
            ss << s1;
            ss >> a1;
            ss >> a2;
            ss >> a3;
        }

        // a2 = a2 + 1;
        // a3 = a3 + 1;
        node_edge noge;
        if (a2 < a3)
        {
            noge.node_idx_1 = a2 - 1;
            noge.node_idx_2 = a3 - 1;
        }
        else
        {
            noge.node_idx_1 = a3 - 1;
            noge.node_idx_2 = a2 - 1;
        }

        // noge.radius = 0.2; // assign a radius for each edge

        node_array[a2 - 1].edge_ids.push_back(node_edges.size());
        node_array[a3 - 1].edge_ids.push_back(node_edges.size());
        node_edges.push_back(noge);
        double len = node_array[a2 - 1].origin.Distance(node_array[a3 - 1].origin);
        if (len < minLength)
            minLength = len;
    }

    if (minLength < 1)
    {
        double scale = 1 / minLength;
        minLength = minLength * scale;
        for (int i = 0; i < node_array.size(); i++)
        {
            node_array[i].origin.SetX(node_array[i].origin.X() * scale);
            node_array[i].origin.SetY(node_array[i].origin.Y() * scale);
            node_array[i].origin.SetZ(node_array[i].origin.Z() * scale);
        }
    }

    for (int i = 0; i < node_edges.size(); i++)
         node_edges[i].radius = minLength / 8.0;
        //node_edges[i].radius = 1;
    std::cout << "strut radius: " << node_edges[0].radius << std::endl;
}

void NodeManager::nodepairCutting(std::vector<node>& nodeArray)
{
    circle cir;
    for (int k = 0; k < nodeArray.size(); k++)
    {
        double sphereRadii = 100000;
        gp_Pnt cpnt = nodeArray[k].origin;
        for (int i = 0; i < nodeArray[k].edge_ids.size(); i++)
        {
            node_edge noge = node_edges[nodeArray[k].edge_ids[i]];
            int theOtehrNode = -1;
            if (noge.node_idx_1 == k)
                theOtehrNode = noge.node_idx_2;
            else
                theOtehrNode = noge.node_idx_1;

            float max_sphereRadius = 0;
            float radius1 = noge.radius;
            gp_Vec vec1(cpnt, gp_Pnt(nodeArray[theOtehrNode].origin.X(), nodeArray[theOtehrNode].origin.Y(), nodeArray[theOtehrNode].origin.Z()));
            for (int j = 0; j < nodeArray[k].edge_ids.size(); j++)
            {
                if (i != j)
                {
                    node_edge temp_noge = node_edges[nodeArray[k].edge_ids[j]];
                    int temp_theOtehrNode = -1;
                    if (temp_noge.node_idx_1 == k)
                        temp_theOtehrNode = temp_noge.node_idx_2;
                    else
                        temp_theOtehrNode = temp_noge.node_idx_1;
                    float radius2 = temp_noge.radius;
                    gp_Vec vec2(cpnt, gp_Pnt(nodeArray[temp_theOtehrNode].origin.X(),
                        nodeArray[temp_theOtehrNode].origin.Y(),
                        nodeArray[temp_theOtehrNode].origin.Z()));

                    if (radius1 > radius2)
                    {
                        float temp = radius1;
                        radius1 = radius2;
                        radius2 = temp;
                    }
                    float mindis = (radius2 + radius1 * std::cos(vec1.Angle(vec2))) / std::sin(vec1.Angle(vec2));
                    float sphereradius = std::sqrt(std::pow(mindis, 2) + std::pow(radius1, 2));
                    if (sphereradius > max_sphereRadius)
                        max_sphereRadius = sphereradius;
                }
            }

            if (max_sphereRadius > 0)
            {
                max_sphereRadius = max_sphereRadius * 1.3;

                if (sphereRadii > max_sphereRadius)
                    sphereRadii = max_sphereRadius;

                float anglex = vec1.Angle(gp_Vec(1, 0, 0));
                float angley = vec1.Angle(gp_Vec(0, 1, 0));
                float anglez = vec1.Angle(gp_Vec(0, 0, 1));
                float disx = max_sphereRadius * std::cos(anglex);
                float disy = max_sphereRadius * std::cos(angley);
                float disz = max_sphereRadius * std::cos(anglez);
                cir.circle_center = gp_Pnt(cpnt.X() + disx, cpnt.Y() + disy, cpnt.Z() + disz);
                cir.circle_radius = noge.radius;

                if (k < theOtehrNode)
                    node_edges[nodeArray[k].edge_ids[i]].node_circle_idx_1 = nodeArray[k].circles.size();
                else
                    node_edges[nodeArray[k].edge_ids[i]].node_circle_idx_2 = nodeArray[k].circles.size();

                nodeArray[k].circles.push_back(cir);
            }
        }
        nodeArray[k].sphereRadii = sphereRadii;
    }
}

void NodeManager::triangulateCylinder_waterTight(std::string& path, int iteration)
{
    std::cout << ">>>>>>Exporting cylinder STL..." << std::endl;
    std::ofstream file;
    file.open(path);
    if (!file)
        std::cout << ">>>>>>>>ERROR::triangulate cylinder failure." << std::endl;
    assert(file);
    file << "solid WRAP" << std::endl;

    for (int i = 0; i < node_edges.size(); i++)
    {
        std::vector<gp_Pnt> cirPoints_1, cirPoints_2;
        gp_Pnt cirCenter_1, cirCenter_2;
        double radii_1, radii_2;

        if (node_edges[i].node_idx_1 < node_edges[i].node_idx_2)
        {
            cirPoints_1 = node_array[node_edges[i].node_idx_1].circles[node_edges[i].node_circle_idx_1].cirPoints;
            cirCenter_1 = node_array[node_edges[i].node_idx_1].circles[node_edges[i].node_circle_idx_1].circle_center;
            radii_1 = node_array[node_edges[i].node_idx_1].circles[node_edges[i].node_circle_idx_1].circle_radius;

            cirPoints_2 = node_array[node_edges[i].node_idx_2].circles[node_edges[i].node_circle_idx_2].cirPoints;
            cirCenter_2 = node_array[node_edges[i].node_idx_2].circles[node_edges[i].node_circle_idx_2].circle_center;
            radii_2 = node_array[node_edges[i].node_idx_2].circles[node_edges[i].node_circle_idx_2].circle_radius;
        }
        else
        {
            cirPoints_1 = node_array[node_edges[i].node_idx_1].circles[node_edges[i].node_circle_idx_2].cirPoints;
            cirCenter_1 = node_array[node_edges[i].node_idx_1].circles[node_edges[i].node_circle_idx_2].circle_center;
            radii_1 = node_array[node_edges[i].node_idx_1].circles[node_edges[i].node_circle_idx_2].circle_radius;

            cirPoints_2 = node_array[node_edges[i].node_idx_2].circles[node_edges[i].node_circle_idx_1].cirPoints;
            cirCenter_2 = node_array[node_edges[i].node_idx_2].circles[node_edges[i].node_circle_idx_1].circle_center;
            radii_2 = node_array[node_edges[i].node_idx_2].circles[node_edges[i].node_circle_idx_1].circle_radius;
        }

        for (int iter = 0; iter < iteration; iter++)
        {
            std::vector<gp_Pnt> temp;
            for (int j = 0; j < cirPoints_1.size(); j++)
            {
                int k = (j + 1) % cirPoints_1.size();
                gp_Vec v1(cirCenter_1, cirPoints_1[j]);
                v1.Normalize();
                gp_Vec v2(cirCenter_1, cirPoints_1[k]);
                v2.Normalize();
                gp_Vec vec = v1 + v2;
                vec.Normalize();
                gp_Pnt newPnt(cirCenter_1.X() + vec.X() * radii_1, cirCenter_1.Y() + vec.Y() * radii_1, cirCenter_1.Z() + vec.Z() * radii_1);
                temp.push_back(cirPoints_1[j]);
                temp.push_back(newPnt);
            }
            cirPoints_1.clear();
            cirPoints_1 = temp;
        }

        for (int iter = 0; iter < iteration; iter++)
        {
            std::vector<gp_Pnt> temp;
            for (int j = 0; j < cirPoints_2.size(); j++)
            {
                int k = (j + 1) % cirPoints_2.size();
                gp_Vec v1(cirCenter_2, cirPoints_2[j]);
                v1.Normalize();
                gp_Vec v2(cirCenter_2, cirPoints_2[k]);
                v2.Normalize();
                gp_Vec vec = v1 + v2;
                vec.Normalize();
                gp_Pnt newPnt(cirCenter_2.X() + vec.X() * radii_2, cirCenter_2.Y() + vec.Y() * radii_2, cirCenter_2.Z() + vec.Z() * radii_2);
                temp.push_back(cirPoints_2[j]);
                temp.push_back(newPnt);
            }
            cirPoints_2.clear();
            cirPoints_2 = temp;
        }

        int n1 = cirPoints_1.size();
        int n2 = cirPoints_2.size();
        std::vector<gp_Pnt> largerCir, smallerCir;
        if (n1 > n2)
        {
            largerCir = cirPoints_1;
            smallerCir = cirPoints_2;
        }
        else
        {
            largerCir = cirPoints_2;
            smallerCir = cirPoints_1;
        }

        int left = 0, right = largerCir.size() - 1;
        while (left < right)
        {
            gp_Pnt tempPnt = largerCir[left];
            largerCir[left] = largerCir[right];
            largerCir[right] = tempPnt;
            left++;
            right--;
        }

        std::vector<int> map;
        findBestIndexMatch(smallerCir, largerCir, map);
        for (int j = 0; j < smallerCir.size(); j++)
        {
            int next_j = (j + 1) % smallerCir.size();
            int k = map[j];
            int next_k = map[next_j];

            int indexnk = next_k;
            if (next_k < k)
                indexnk = largerCir.size() + next_k;

            if (indexnk - k > 1)
            {
                while (k < indexnk && k + 1 <= indexnk - 1)
                {
                    writeTriangle(file, smallerCir[j], largerCir[k % largerCir.size()], largerCir[(k + 1) % largerCir.size()]);
                    writeTriangle(file, smallerCir[next_j], largerCir[(indexnk - 1) % largerCir.size()], largerCir[indexnk % largerCir.size()]);
                    k++;
                    indexnk--;
                }

                if (k == indexnk)
                {
                    writeTriangle(file, smallerCir[j], largerCir[k % largerCir.size()], smallerCir[next_j]);
                }
                else
                {
                    writeTriangle(file, smallerCir[j], largerCir[(k) % largerCir.size()], largerCir[(indexnk) % largerCir.size()]);
                    writeTriangle(file, smallerCir[j], largerCir[(indexnk) % largerCir.size()], smallerCir[next_j]);
                }
            }
            else
            {
                writeTriangle(file, smallerCir[j], largerCir[k], largerCir[next_k]);
                writeTriangle(file, smallerCir[j], largerCir[next_k], smallerCir[next_j]);
            }
        }
    }
    file << "endsolid WRAP" << "\n";
    file.close();
    std::cout << ">>>>>>The cylinder STL has been exported." << std::endl;
}

void NodeManager::writeTriangle(std::ofstream& file, const gp_Pnt& p1, const gp_Pnt& p2, const gp_Pnt& p3)
{
    // gp_Vec v1(p1, p2);
    // gp_Vec v2(p1, p3);
    // v1.Cross(v2);
    // v1.Normalize();
    file << "facet normal " << 0 << " " << 0 << " " << 1 << "\n";
    file << "outer loop" << "\n";
    file << "vertex " << p1.X() << " " << p1.Y() << " " << p1.Z() << std::endl
        << "vertex " << p2.X() << " " << p2.Y() << " " << p2.Z() << std::endl
        << "vertex " << p3.X() << " " << p3.Y() << " " << p3.Z() << "\n";
    file << "endloop" << "\n";
    file << "endfacet" << "\n";
}

void NodeManager::findBestIndexMatch(const std::vector<gp_Pnt>& smallerCircle, const std::vector<gp_Pnt>& largerCircle, std::vector<int>& mapping)
{
    int n1 = smallerCircle.size();
    int n2 = largerCircle.size();
    int bestShift = 0;
    double minTotalDistance = std::numeric_limits<double>::max();

    for (int shift = 0; shift < n2; ++shift)
    {
        double totalDistance = 0.0;
        std::vector<int> tempMapping(n1);

        // Map points from the smaller set to the larger set using interpolation
        for (int i = 0; i < n1; ++i)
        {
            int j = static_cast<int>((static_cast<double>(i) / n1) * n2 + shift) % n2;
            totalDistance += smallerCircle[i].Distance(largerCircle[j]); // Use gp_Pnt::Distance()
            tempMapping[i] = j;
        }

        // Update best shift if a new minimum distance is found
        if (totalDistance < minTotalDistance)
        {
            minTotalDistance = totalDistance;
            bestShift = shift;
            mapping = tempMapping;
        }
    }
}

bool NodeManager::checkCoplanar(node& _node)
{
    gp_Vec v1(_node.origin, _node.circles[0].circle_center);
    gp_Vec v2(_node.origin, _node.circles[1].circle_center);
    gp_Vec crossv = v1.Crossed(v2);
    for (int i = 2; i < _node.circles.size(); i++)
    {
        gp_Vec v3(_node.origin, _node.circles[i].circle_center);
        if (abs(90 - crossv.Angle(v3) / M_PI * 180) > 2)
            return false;
    }
    return true;
}