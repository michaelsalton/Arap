#include "app/overlay_renderer.h"
#include "types/modes.h"
#include "types/colors.h"
#include <igl/per_vertex_normals.h>

using namespace colors;

void OverlayRenderer::update(igl::opengl::ViewerData& data,
                             const Eigen::MatrixXd& V_current,
                             const Eigen::MatrixXi& F,
                             const Eigen::MatrixXi& E,
                             const std::set<int>& selected,
                             int selection_mode,
                             int dragged_vertex) {
    data.clear_points();
    data.clear_edges();

    Eigen::MatrixXd V_overlay = compute_overlay_positions(V_current, F);

    // Reset per-face colors when not in face mode
    if (selection_mode != FACE) {
        Eigen::MatrixXd C(1, 3);
        C.row(0) = kDefaultFace;
        data.set_colors(C);
    }

    switch (selection_mode) {
        case VERTEX: highlight_vertices(data, V_overlay, selected, dragged_vertex); break;
        case EDGE:   highlight_edges(data, V_overlay, E, selected, dragged_vertex); break;
        case FACE:   highlight_faces(data, V_overlay, F, selected, dragged_vertex); break;
    }
}

Eigen::MatrixXd OverlayRenderer::compute_overlay_positions(const Eigen::MatrixXd& V,
                                                           const Eigen::MatrixXi& F) {
    Eigen::MatrixXd N;
    igl::per_vertex_normals(V, F, N);
    double diag = (V.colwise().maxCoeff() - V.colwise().minCoeff()).norm();
    return V + N * (diag * 0.002);
}

void OverlayRenderer::highlight_vertices(igl::opengl::ViewerData& data,
                                         const Eigen::MatrixXd& V_overlay,
                                         const std::set<int>& selected,
                                         int dragged_vertex) {
    for (int i = 0; i < V_overlay.rows(); ++i) {
        if (selected.count(i)) {
            Eigen::RowVector3d color = (i == dragged_vertex) ? kGreen : kRed;
            data.add_points(V_overlay.row(i), color);
        } else {
            data.add_points(V_overlay.row(i), kOrange);
        }
    }
}

void OverlayRenderer::highlight_edges(igl::opengl::ViewerData& data,
                                      const Eigen::MatrixXd& V_overlay,
                                      const Eigen::MatrixXi& E,
                                      const std::set<int>& selected,
                                      int dragged_vertex) {
    int ne = E.rows();
    Eigen::MatrixXd P1(ne, 3), P2(ne, 3), C(ne, 3);
    for (int e = 0; e < ne; ++e) {
        int i = E(e, 0), j = E(e, 1);
        P1.row(e) = V_overlay.row(i);
        P2.row(e) = V_overlay.row(j);
        bool edge_selected = selected.count(i) && selected.count(j);
        C.row(e) = edge_selected ? kRed : kOrange;
    }
    data.add_edges(P1, P2, C);

    for (int vid : selected) {
        Eigen::RowVector3d color = (vid == dragged_vertex) ? kGreen : kRed;
        data.add_points(V_overlay.row(vid), color);
    }
}

void OverlayRenderer::highlight_faces(igl::opengl::ViewerData& data,
                                      const Eigen::MatrixXd& V_overlay,
                                      const Eigen::MatrixXi& F,
                                      const std::set<int>& selected,
                                      int dragged_vertex) {
    Eigen::MatrixXd C(F.rows(), 3);
    for (int f = 0; f < F.rows(); ++f) {
        bool all_selected = selected.count(F(f, 0))
                         && selected.count(F(f, 1))
                         && selected.count(F(f, 2));
        C.row(f) = all_selected ? kRed : kOrange;
    }
    data.set_colors(C);

    for (int vid : selected) {
        Eigen::RowVector3d color = (vid == dragged_vertex) ? kGreen : kRed;
        data.add_points(V_overlay.row(vid), color);
    }
}
