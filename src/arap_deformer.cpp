#include "arap_deformer.h"
#include <igl/cotmatrix.h>
#include <igl/adjacency_list.h>

void ArapDeformer::precompute(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) {
    V_orig_ = V;
    F_ = F;
    n_verts_ = V.rows();

    // 1. Cotangent Laplacian (n x n sparse matrix)
    igl::cotmatrix(V, F, L_);

    // 2. Differential coordinates (n x 3)
    deltas_ = L_ * V;

    // 3. Per-vertex adjacency (1-ring neighbors)
    igl::adjacency_list(F, neighbors_);

    precomputed_ = true;
}

void ArapDeformer::set_constraints(const Eigen::VectorXi& constraint_indices) {
    constraint_indices_ = constraint_indices;
    int num_constraints = constraint_indices_.size();

    // Build augmented matrix A = [L; W] of size (n + |C|) x n
    std::vector<Eigen::Triplet<double>> triplets;

    // Copy L entries
    for (int k = 0; k < L_.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(L_, k); it; ++it)
            triplets.emplace_back(it.row(), it.col(), it.value());

    // Constraint rows
    for (int k = 0; k < num_constraints; ++k)
        triplets.emplace_back(n_verts_ + k, constraint_indices_[k], constraint_weight_);

    A_.resize(n_verts_ + num_constraints, n_verts_);
    A_.setFromTriplets(triplets.begin(), triplets.end());

    // Prefactor A^T * A
    AtA_ = A_.transpose() * A_;
    solver_.compute(AtA_);
    prefactored_ = (solver_.info() == Eigen::Success);
}

Eigen::MatrixXd ArapDeformer::solve_laplacian(const Eigen::MatrixXd& constraint_positions) {
    if (!prefactored_) return V_orig_;

    int num_constraints = constraint_indices_.size();

    // Build RHS: b = [deltas; W * target_positions]
    Eigen::MatrixXd b(n_verts_ + num_constraints, 3);
    b.topRows(n_verts_) = deltas_;
    for (int k = 0; k < num_constraints; ++k)
        b.row(n_verts_ + k) = constraint_weight_ * constraint_positions.row(k);

    Eigen::MatrixXd Atb = A_.transpose() * b;

    // Solve each coordinate independently
    Eigen::MatrixXd V_prime(n_verts_, 3);
    for (int d = 0; d < 3; ++d)
        V_prime.col(d) = solver_.solve(Atb.col(d));

    return V_prime;
}

Eigen::MatrixXd ArapDeformer::solve_arap(const Eigen::MatrixXd& constraint_positions,
                                          int num_iterations) {
    // TODO: Feature 04
    return V_orig_;
}

void ArapDeformer::compute_rotations(const Eigen::MatrixXd& V_deformed,
                                      std::vector<Eigen::Matrix3d>& rotations) {
    // TODO: Feature 04
}

Eigen::MatrixXd ArapDeformer::build_arap_rhs(const std::vector<Eigen::Matrix3d>& rotations) {
    // TODO: Feature 04
    return deltas_;
}
