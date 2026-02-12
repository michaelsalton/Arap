#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <vector>

class ArapDeformer {
public:
    // Precompute Laplacian, differential coordinates, and adjacency from mesh
    void precompute(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);

    // Set constraint vertex indices and prefactor the system matrix
    void set_constraints(const Eigen::VectorXi& constraint_indices);

    // Solve Laplacian deformation given target positions for constrained vertices
    Eigen::MatrixXd solve_laplacian(const Eigen::MatrixXd& constraint_positions);

    // Solve ARAP deformation (iterative local/global)
    Eigen::MatrixXd solve_arap(const Eigen::MatrixXd& constraint_positions,
                               int num_iterations = 5);

    bool is_precomputed() const { return precomputed_; }
    bool is_prefactored() const { return prefactored_; }
    int num_vertices() const { return n_verts_; }

private:
    // Compute best-fit rotations for all vertices
    void compute_rotations(const Eigen::MatrixXd& V_deformed,
                           std::vector<Eigen::Matrix3d>& rotations);

    // Build rotation-aware RHS for the ARAP global step
    Eigen::MatrixXd build_arap_rhs(const std::vector<Eigen::Matrix3d>& rotations);

    // Original mesh
    Eigen::MatrixXd V_orig_;
    Eigen::MatrixXi F_;
    int n_verts_ = 0;

    // Laplacian
    Eigen::SparseMatrix<double> L_;
    Eigen::MatrixXd deltas_;

    // Adjacency: neighbors_[i] = list of vertex indices adjacent to i
    std::vector<std::vector<int>> neighbors_;

    // Constraint data
    Eigen::VectorXi constraint_indices_;
    double constraint_weight_ = 1.0;

    // Augmented system (precomputed on constraint change)
    Eigen::SparseMatrix<double> A_;
    Eigen::SparseMatrix<double> AtA_;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver_;

    bool precomputed_ = false;
    bool prefactored_ = false;
};
