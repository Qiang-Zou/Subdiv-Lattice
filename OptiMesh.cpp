#include "OptiMesh.h"

const double DOUBLE_INF = 1e8;

OptiMesh::OptiMesh(MyMesh& _myMesh) : myMesh(_myMesh)
{
}

OptiMesh::~OptiMesh()
{
}

void OptiMesh::laplace_matrix(MyMesh& myMesh, SparseMatrix& L, bool clamp)
{
    const int nv = myMesh.mesh.n_vertices();
    const int nf = myMesh.mesh.n_faces();
    std::vector<int> vertices; // polygon vertices
    DenseMatrix polygon;       // positions of polygon vertices
    DenseMatrix Lpoly;         // local laplace matrix

    std::vector<Triplet> triplets;
    triplets.reserve(9 * nf); // estimate for triangle meshes

    for (Mesh::FaceIter fiter = myMesh.mesh.faces_begin(); fiter != myMesh.mesh.faces_end(); ++fiter)
    {
        Mesh::FaceHandle f = *fiter;
        // collect polygon vertices
        vertices.clear();
        for (Mesh::FaceVertexIter fv_it = myMesh.mesh.fv_iter(f); fv_it.is_valid(); ++fv_it)
        {
            Mesh::VertexHandle vh = *fv_it;
            vertices.push_back(vh.idx());
        }

        int n = vertices.size();

        // collect their positions
        polygon.resize(n, 3);
        int vidx = 0;
        for (Mesh::FaceVertexIter fv_it = myMesh.mesh.fv_iter(f); fv_it.is_valid(); ++fv_it)
        {
            Mesh::Point point = myMesh.mesh.point(*fv_it); // Get the point (vertex coordinates)
            polygon.row(vidx) = Eigen::Vector3d(point[0], point[1], point[2]);
            vidx++;
        }

        // setup local laplace matrix
        polygon_laplace_matrix(polygon, Lpoly);

        // assemble to global laplace matrix
        for (int j = 0; j < n; ++j)
        {
            for (int k = 0; k < n; ++k)
            {
                triplets.emplace_back(vertices[k], vertices[j],
                    -Lpoly(k, j));
            }
        }
    }

    // build sparse matrix from triplets
    L.resize(nv, nv);

    L.setFromTriplets(triplets.begin(), triplets.end());

    // clamp negative off-diagonal entries to zero
    if (clamp)
    {
        for (unsigned int k = 0; k < L.outerSize(); k++)
        {
            double diag_offset(0.0);

            for (SparseMatrix::InnerIterator iter(L, k); iter; ++iter)
            {
                if (iter.row() != iter.col() && iter.value() < 0.0)
                {
                    diag_offset += -iter.value();
                    iter.valueRef() = 0.0;
                }
            }
            for (SparseMatrix::InnerIterator iter(L, k); iter; ++iter)
            {
                if (iter.row() == iter.col() && iter.value() < 0.0)
                    iter.valueRef() -= diag_offset;
            }
        }
    }
}

void OptiMesh::polygon_laplace_matrix(const DenseMatrix& polygon, DenseMatrix& Lpoly)
{
    const int n = (int)polygon.rows();
    Lpoly = DenseMatrix::Zero(n, n);
    triangle_laplace_matrix(polygon.row(0), polygon.row(1), polygon.row(2),
        Lpoly);
}

void OptiMesh::triangle_laplace_matrix(const Eigen::Vector3d& p0,
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2, DenseMatrix& Ltri)
{
    std::array<double, 3> l, l2, cot;

    // squared edge lengths
    l2[0] = (p1 - p2).squaredNorm();
    l2[1] = (p0 - p2).squaredNorm();
    l2[2] = (p0 - p1).squaredNorm();

    // edge lengths
    l[0] = sqrt(l2[0]);
    l[1] = sqrt(l2[1]);
    l[2] = sqrt(l2[2]);

    // triangle area
    const double arg = (l[0] + (l[1] + l[2])) * (l[2] - (l[0] - l[1])) *
        (l[2] + (l[0] - l[1])) * (l[0] + (l[1] - l[2]));
    const double area = 0.5 * sqrt(arg);

    if (area > std::numeric_limits<double>::min())
    {
        // cotangents of angles
        cot[0] = 0.25 * (l2[1] + l2[2] - l2[0]) / area;
        cot[1] = 0.25 * (l2[2] + l2[0] - l2[1]) / area;
        cot[2] = 0.25 * (l2[0] + l2[1] - l2[2]) / area;

        Ltri(0, 0) = cot[1] + cot[2];
        Ltri(1, 1) = cot[0] + cot[2];
        Ltri(2, 2) = cot[0] + cot[1];
        Ltri(1, 0) = Ltri(0, 1) = -cot[2];
        Ltri(2, 0) = Ltri(0, 2) = -cot[1];
        Ltri(2, 1) = Ltri(1, 2) = -cot[0];
    }
}

void OptiMesh::laplacianFairing(int iteration)
{
    int k = 3;
    for (int i = 0; i < iteration; i++)
    {
        const int n = myMesh.mesh.n_vertices();

        // lock k locked boundary rings
        for (auto v : myMesh.mesh.vertices())
        {
            // lock boundary
            if (myMesh.mesh.is_boundary(v))
            {
                myMesh.vprop[v.idx()].vlocked = true;

                // lock one-ring of boundary
                if (k > 1)
                {
                    for (auto vv_it = myMesh.mesh.vv_iter(v); vv_it.is_valid(); ++vv_it)
                    {
                        auto vv = *vv_it;
                        myMesh.vprop[vv.idx()].vlocked = true;

                        // lock two-ring of boundary
                        if (k > 2)
                        {
                            for (auto vvv_it = myMesh.mesh.vv_iter(vv); vvv_it.is_valid(); ++vvv_it)
                            {
                                auto vvv = *vvv_it;
                                myMesh.vprop[vvv.idx()].vlocked = true;

                                if (myMesh.vprop[vvv.idx()].isHandled == false)
                                    myMesh.vprop[vvv.idx()].vlocked = false;

                                if (k > 3)
                                {
                                    for (auto vvvv_it = myMesh.mesh.vv_iter(vvv); vvvv_it.is_valid(); ++vvvv_it)
                                    {
                                        auto vvvv = *vvvv_it;
                                        myMesh.vprop[vvvv.idx()].vlocked = true;

                                        if (k > 4)
                                        {
                                            for (auto vvvvv_it = myMesh.mesh.vv_iter(vvvv); vvvvv_it.is_valid(); ++vvvvv_it)
                                            {
                                                auto vvvvv = *vvvvv_it;
                                                myMesh.vprop[vvvvv.idx()].vlocked = true;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // build (zero) right-hand side B
        DenseMatrix B(n, 3);
        B.setZero();

        // positions will be used as constraints
        DenseMatrix X;
        coordinates_to_matrix(myMesh.mesh, X);

        // build matrix
        laplace_matrix(myMesh, L, true);

        DiagonalMatrix M;
        mass_matrix(myMesh.mesh, M);
        DiagonalMatrix invM = M.inverse();

        SparseMatrix A = L;
        for (unsigned int i = 1; i < 2; ++i) // fixed bi-laplacian
            A = L * invM * A;
        B = M * B;

        // solve system
        auto is_locked = [&](unsigned int i)
        { return myMesh.vprop[i].vlocked; };

        X = cholesky_solve(A, B, is_locked, X);

        // copy solution
        matrix_to_coordinates(X, myMesh.mesh);
    }
}

////modified from pmp
DenseMatrix OptiMesh::cholesky_solve(
    const SparseMatrix& A, const DenseMatrix& B,
    const std::function<bool(unsigned int)>& is_constrained,
    const DenseMatrix& C)
{
    // if nothing is fixed, then use unconstrained solve
    int n_constraints(0);
    for (int i = 0; i < A.cols(); ++i)
        if (is_constrained(i))
            ++n_constraints;
    // if (!n_constraints)
    //     return cholesky_solve(A, B);

    // build index map; n is #dofs
    int n = 0;
    std::vector<int> idx(A.cols(), -1);
    for (int i = 0; i < A.cols(); ++i)
        if (!is_constrained(i))
            idx[i] = n++;

    // copy columns for rhs
    DenseMatrix BB(n, B.cols());
    for (int i = 0; i < A.cols(); ++i)
        if (idx[i] != -1)
            BB.row(idx[i]) = B.row(i);

    // collect entries for reduced matrix
    // update rhs with constraints
    std::vector<Triplet> triplets;
    triplets.reserve(A.nonZeros());
    for (unsigned int k = 0; k < A.outerSize(); k++)
    {
        for (SparseMatrix::InnerIterator iter(A, k); iter; ++iter)
        {
            const int i = iter.row();
            const int j = iter.col();

            if (idx[i] != -1) // row is dof
            {
                if (idx[j] != -1) // col is dof
                {
                    triplets.emplace_back(idx[i], idx[j], iter.value());
                }
                else // col is constraint
                {
                    BB.row(idx[i]) -= iter.value() * C.row(j);
                }
            }
        }
    }
    SparseMatrix AA(n, n);
    AA.setFromTriplets(triplets.begin(), triplets.end());

    // factorize system
    Eigen::SimplicialLDLT<SparseMatrix> solver;
    solver.compute(AA);
    if (solver.info() != Eigen::Success)
    {
        std::cout << "cholesky_solve: Failed to factorize linear system." << std::endl;
    }

    // solve system
    const DenseMatrix XX = solver.solve(BB);
    if (solver.info() != Eigen::Success)
    {
        std::cout << "cholesky_solve: Failed to solve linear system." << std::endl;
    }

    // build full-size result vector from solver result (X) and constraints (C)
    DenseMatrix X(B.rows(), B.cols());

    for (int i = 0; i < A.cols(); ++i)
        X.row(i) = idx[i] == -1 ? C.row(i) : XX.row(idx[i]);
    
    return X;
}

void OptiMesh::coordinates_to_matrix(Mesh& mesh, DenseMatrix& X)
{
    X.resize(mesh.n_vertices(), 3);
    for (auto v : mesh.vertices())
        X.row(v.idx()) = Eigen::Vector3d(mesh.point(v)[0], mesh.point(v)[1], mesh.point(v)[2]);
}

void OptiMesh::matrix_to_coordinates(const DenseMatrix& X, Mesh& mesh)
{
    assert((size_t)X.rows() == mesh.n_vertices() && X.cols() == 3);
    for (auto v : mesh.vertices())
    {
        Mesh::Point pnt(X.row(v.idx())[0], X.row(v.idx())[1], X.row(v.idx())[2]);
        mesh.set_point(v, pnt);
    }
}

void OptiMesh::triangle_mass_matrix(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2, DiagonalMatrix& Mtri)
{
    // three vertex positions
    const std::array<Eigen::Vector3d, 3> p = { p0, p1, p2 };

    // edge vectors
    std::array<Eigen::Vector3d, 3> e;
    for (int i = 0; i < 3; ++i)
        e[i] = p[(i + 1) % 3] - p[i];

    // compute and check (twice the) triangle area
    const auto tri_area = (e[0].cross(e[1])).norm();

    if (tri_area <= std::numeric_limits<double>::min())
    {
        Mtri.setZero(3);
        return;
    }

    // dot products for each corner (of its two emanating edge vectors)
    std::array<double, 3> d;
    for (int i = 0; i < 3; ++i)
        d[i] = -e[i].dot(e[(i + 2) % 3]);

    // cotangents for each corner: cot = cos/sin = dot(A,B)/norm(cross(A,B))
    std::array<double, 3> cot;
    for (int i = 0; i < 3; ++i)
        cot[i] = d[i] / tri_area;

    // compute area for each corner
    Eigen::Vector3d area;
    for (int i = 0; i < 3; ++i)
    {
        // angle at corner is obtuse
        if (d[i] < 0.0)
        {
            area[i] = 0.25 * tri_area;
        }
        // angle at some other corner is obtuse
        else if (d[(i + 1) % 3] < 0.0 || d[(i + 2) % 3] < 0.0)
        {
            area[i] = 0.125 * tri_area;
        }
        // no obtuse angles
        else
        {
            area[i] = 0.125 * (((e[i]) * cot[(i + 2) % 3]).squaredNorm() +
                ((e[(i + 2) % 3]) * cot[(i + 1) % 3]).squaredNorm());
        }
    }

    Mtri = area.asDiagonal();
}

void OptiMesh::polygon_mass_matrix(const DenseMatrix& polygon, DiagonalMatrix& Mpoly)
{
    triangle_mass_matrix(polygon.row(0), polygon.row(1), polygon.row(2),
        Mpoly);
    return;
}

void OptiMesh::mass_matrix(Mesh& mesh, DiagonalMatrix& M)
{
    const int nv = mesh.n_vertices();
    std::vector<Mesh::VertexHandle> vertices; // polygon vertices
    DenseMatrix polygon;                      // positions of polygon vertices
    DiagonalMatrix Mpoly;                     // local mass matrix

    M.setZero(nv);

    for (auto f : mesh.faces())
    {
        // collect polygon vertices
        vertices.clear();
        for (auto fv_it = mesh.fv_iter(f); fv_it.is_valid(); ++fv_it)
        {
            auto v = *fv_it; // 'v' is a vertex handle
            vertices.push_back(v);
        }
        const int n = vertices.size();

        // collect their positions
        polygon.resize(n, 3);
        for (int i = 0; i < n; ++i)
        {
            polygon.row(i) = Eigen::Vector3d(mesh.point(vertices[i])[0], mesh.point(vertices[i])[1], mesh.point(vertices[i])[2]);
        }

        // setup local mass matrix
        polygon_mass_matrix(polygon, Mpoly);

        // assemble to global mass matrix
        for (int k = 0; k < n; ++k)
        {
            M.diagonal()[vertices[k].idx()] += Mpoly.diagonal()[k];
        }
    }
}