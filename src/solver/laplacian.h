#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

class LaplacianSolver {
public:
    virtual ~LaplacianSolver() = default;

    void precompute(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);
    void set_constraints(const Eigen::VectorXi& constraint_indices);
    Eigen::MatrixXd solve(const Eigen::MatrixXd& constraint_positions);

    bool is_precomputed() const { return precomputed_; }
    bool is_prefactored() const { return prefactored_; }
    int num_vertices() const { return n_verts_; }

protected:
    Eigen::MatrixXd V_orig_;
    Eigen::MatrixXi F_;
    int n_verts_ = 0;

    Eigen::SparseMatrix<double> L_;
    Eigen::MatrixXd deltas_;

    Eigen::VectorXi constraint_indices_;
    double constraint_weight_ = 1.0;

    Eigen::SparseMatrix<double> A_;
    Eigen::SparseMatrix<double> AtA_;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver_;

    bool precomputed_ = false;
    bool prefactored_ = false;
};
