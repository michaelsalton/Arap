#include "viewer_app.h"
#include <igl/read_triangle_mesh.h>
#include <igl/unproject_onto_mesh.h>
#include <iostream>

bool ViewerApp::load_mesh(const std::string& path) {
    if (!igl::read_triangle_mesh(path, V_, F_)) {
        std::cerr << "Error: could not read mesh file: " << path << std::endl;
        return false;
    }

    if (V_.rows() == 0 || F_.rows() == 0) {
        std::cerr << "Warning: mesh is empty (" << V_.rows() << " vertices, "
                  << F_.rows() << " faces)" << std::endl;
        return false;
    }

    V_current_ = V_;

    std::cout << "Loaded mesh: " << V_.rows() << " vertices, "
              << F_.rows() << " faces" << std::endl;

    // Precompute deformer data
    deformer_.precompute(V_, F_);

    return true;
}

void ViewerApp::launch() {
    viewer_.data().set_mesh(V_current_, F_);
    viewer_.data().compute_normals();
    viewer_.data().point_size = 10.0f;

    viewer_.callback_mouse_down =
        [this](igl::opengl::glfw::Viewer& viewer, int button, int modifier) -> bool {
        // Only handle left click in select mode
        if (button != 0 || interaction_mode_ != 0) return false;

        // Ray-cast onto mesh
        int fid;
        Eigen::Vector3f bc;
        Eigen::Vector2f pos(viewer.current_mouse_x,
                            viewer.core().viewport(3) - viewer.current_mouse_y);

        if (!igl::unproject_onto_mesh(
                pos, viewer.core().view, viewer.core().proj,
                viewer.core().viewport, V_current_, F_, fid, bc)) {
            return false;  // missed the mesh
        }

        // Pick nearest vertex via max barycentric coordinate
        int vid = F_(fid, 0);
        if (bc(1) > bc(0) && bc(1) > bc(2)) vid = F_(fid, 1);
        if (bc(2) > bc(0) && bc(2) > bc(1)) vid = F_(fid, 2);

        // Toggle selection
        if (selected_vertices_.count(vid))
            selected_vertices_.erase(vid);
        else
            selected_vertices_.insert(vid);

        sync_constraints();
        update_overlay();
        return true;
    };

    viewer_.launch();
}

void ViewerApp::update_overlay() {
    viewer_.data().clear_points();

    for (int vid : selected_vertices_) {
        Eigen::RowVector3d color(1.0, 0.0, 0.0);  // red for anchors
        if (vid == dragged_vertex_)
            color = Eigen::RowVector3d(0.0, 1.0, 0.0);  // green for dragged
        viewer_.data().add_points(V_current_.row(vid), color);
    }
}

void ViewerApp::sync_constraints() {
    if (!deformer_.is_precomputed()) return;

    Eigen::VectorXi indices(selected_vertices_.size());
    int k = 0;
    for (int vid : selected_vertices_)
        indices(k++) = vid;

    deformer_.set_constraints(indices);
}
