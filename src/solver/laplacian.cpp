#include "solver/laplacian.h"
#include <igl/cotmatrix.h>

void LaplacianSolver::precompute(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) {
    V_orig_ = V;
    F_ = F;
    n_verts_ = V.rows();

    igl::cotmatrix(V, F, L_);
    deltas_ = L_ * V;

    precomputed_ = true;
}

void LaplacianSolver::set_constraints(const Eigen::VectorXi& constraint_indices) {
    constraint_indices_ = constraint_indices;
    int num_constraints = constraint_indices_.size();

    std::vector<Eigen::Triplet<double>> triplets;

    for (int k = 0; k < L_.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(L_, k); it; ++it)
            triplets.emplace_back(it.row(), it.col(), it.value());

    for (int k = 0; k < num_constraints; ++k)
        triplets.emplace_back(n_verts_ + k, constraint_indices_[k], constraint_weight_);

    A_.resize(n_verts_ + num_constraints, n_verts_);
    A_.setFromTriplets(triplets.begin(), triplets.end());

    AtA_ = A_.transpose() * A_;
    solver_.compute(AtA_);
    prefactored_ = (solver_.info() == Eigen::Success);
}

Eigen::MatrixXd LaplacianSolver::solve(const Eigen::MatrixXd& constraint_positions) {
    if (!prefactored_) return V_orig_;

    int num_constraints = constraint_indices_.size();

    Eigen::MatrixXd b(n_verts_ + num_constraints, 3);
    b.topRows(n_verts_) = deltas_;
    for (int k = 0; k < num_constraints; ++k)
        b.row(n_verts_ + k) = constraint_weight_ * constraint_positions.row(k);

    Eigen::MatrixXd Atb = A_.transpose() * b;

    Eigen::MatrixXd V_prime(n_verts_, 3);
    for (int d = 0; d < 3; ++d)
        V_prime.col(d) = solver_.solve(Atb.col(d));

    return V_prime;
}
