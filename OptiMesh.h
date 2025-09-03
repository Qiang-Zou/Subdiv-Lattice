#pragma once

#include "VisualTool.h"

class OptiMesh
{
public:
    void laplace_matrix(MyMesh& myMesh, SparseMatrix& L, bool clamp = false);
    void polygon_laplace_matrix(const DenseMatrix& polygon, DenseMatrix& Lpoly);
    void triangle_laplace_matrix(const Eigen::Vector3d& p0,
        const Eigen::Vector3d& p1,
        const Eigen::Vector3d& p2, DenseMatrix& Ltri);

public: // modified from pmp
    DenseMatrix cholesky_solve(
        const SparseMatrix& A, const DenseMatrix& B,
        const std::function<bool(unsigned int)>& is_constrained,
        const DenseMatrix& C);
    void coordinates_to_matrix(Mesh& mesh, DenseMatrix& X);
    void matrix_to_coordinates(const DenseMatrix& X, Mesh& mesh);
    void laplacianFairing(int iteration);
    void mass_matrix(Mesh& mesh, DiagonalMatrix& M);
    void polygon_mass_matrix(const DenseMatrix& polygon, DiagonalMatrix& Mpoly);
    void triangle_mass_matrix(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1,
        const Eigen::Vector3d& p2, DiagonalMatrix& Mtri);

private:
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    Eigen::Matrix3Xd V;
    SparseMatrix L;
    SparseMatrix Ls;
    DiagonalMatrix M;
    MyMesh& myMesh;

public:
    OptiMesh(MyMesh& _myMesh);
    ~OptiMesh();
};
