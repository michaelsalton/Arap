#pragma once

#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/Core>
#include <set>

class OverlayRenderer {
public:
    void update(igl::opengl::ViewerData& data,
                const Eigen::MatrixXd& V_current,
                const Eigen::MatrixXi& F,
                const Eigen::MatrixXi& E,
                const std::set<int>& selected,
                int selection_mode,
                int dragged_vertex);

private:
    void highlight_vertices(igl::opengl::ViewerData& data,
                            const Eigen::MatrixXd& V_overlay,
                            const std::set<int>& selected,
                            int dragged_vertex);

    void highlight_edges(igl::opengl::ViewerData& data,
                         const Eigen::MatrixXd& V_overlay,
                         const Eigen::MatrixXi& E,
                         const std::set<int>& selected,
                         int dragged_vertex);

    void highlight_faces(igl::opengl::ViewerData& data,
                         const Eigen::MatrixXd& V_overlay,
                         const Eigen::MatrixXi& F,
                         const std::set<int>& selected,
                         int dragged_vertex);

    Eigen::MatrixXd compute_overlay_positions(const Eigen::MatrixXd& V,
                                              const Eigen::MatrixXi& F);
};
