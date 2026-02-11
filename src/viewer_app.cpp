#include "viewer_app.h"
#include <igl/read_triangle_mesh.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/project.h>
#include <igl/unproject.h>
#include <iostream>
#include <chrono>

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

    // --- ImGui setup ---
    viewer_.plugins.push_back(&imgui_plugin_);
    imgui_plugin_.widgets.push_back(&imgui_menu_);

    imgui_menu_.callback_draw_viewer_menu = []() {};  // hide default menu

    panel_.interaction_mode = &interaction_mode_;
    panel_.solver_mode = &solver_mode_;
    panel_.arap_iterations = &arap_iterations_;
    panel_.on_reset_mesh = [this]() { reset_mesh(); };
    panel_.on_clear_selection = [this]() { clear_selection(); };

    imgui_menu_.callback_draw_custom_window = [this]() {
        panel_.num_vertices = V_.rows();
        panel_.num_faces = F_.rows();
        panel_.num_constraints = selected_vertices_.size();
        panel_.last_solve_time_ms = last_solve_time_ms_;
        panel_.draw();
    };

    // --- Keyboard shortcuts ---
    viewer_.callback_key_pressed =
        [this](igl::opengl::glfw::Viewer&, unsigned int key, int) -> bool {
        switch (key) {
            case ' ':
                interaction_mode_ = 1 - interaction_mode_;
                return true;
            case 'r': case 'R':
                reset_mesh();
                return true;
            case '1':
                solver_mode_ = 0;
                return true;
            case '2':
                solver_mode_ = 1;
                return true;
            default:
                return false;
        }
    };

    // --- Mouse Down: select mode or drag start ---
    viewer_.callback_mouse_down =
        [this](igl::opengl::glfw::Viewer& viewer, int button, int modifier) -> bool {
        if (button != 0) return false;  // left click only

        // Ray-cast onto mesh
        int fid;
        Eigen::Vector3f bc;
        Eigen::Vector2f pos(viewer.current_mouse_x,
                            viewer.core().viewport(3) - viewer.current_mouse_y);

        if (!igl::unproject_onto_mesh(
                pos, viewer.core().view, viewer.core().proj,
                viewer.core().viewport, V_current_, F_, fid, bc)) {
            return false;
        }

        // Pick nearest vertex
        int vid = F_(fid, 0);
        if (bc(1) > bc(0) && bc(1) > bc(2)) vid = F_(fid, 1);
        if (bc(2) > bc(0) && bc(2) > bc(1)) vid = F_(fid, 2);

        if (interaction_mode_ == 0) {
            // Select mode: toggle vertex
            if (selected_vertices_.count(vid))
                selected_vertices_.erase(vid);
            else
                selected_vertices_.insert(vid);
            sync_constraints();
            update_overlay();
            return true;
        } else {
            // Drag mode: start drag if vertex is selected
            if (!selected_vertices_.count(vid)) return false;

            is_dragging_ = true;
            dragged_vertex_ = vid;

            // Project vertex to screen to get its depth
            Eigen::Vector3f screen_pos = igl::project(
                Eigen::Vector3f(V_current_.row(vid).cast<float>()),
                viewer.core().view, viewer.core().proj, viewer.core().viewport);
            drag_depth_ = screen_pos(2);

            update_overlay();
            return true;
        }
    };

    // --- Mouse Move: drag vertex ---
    viewer_.callback_mouse_move =
        [this](igl::opengl::glfw::Viewer& viewer, int mouse_x, int mouse_y) -> bool {
        if (!is_dragging_) return false;

        // Unproject current mouse position at the saved depth
        float screen_y = viewer.core().viewport(3) - static_cast<float>(mouse_y);
        Eigen::Vector3f new_pos_3d = igl::unproject(
            Eigen::Vector3f(static_cast<float>(mouse_x), screen_y, drag_depth_),
            viewer.core().view, viewer.core().proj, viewer.core().viewport);

        // Update constraint position for the dragged vertex
        // Find index of dragged_vertex_ in constraint_positions_
        int k = 0;
        for (int vid : selected_vertices_) {
            if (vid == dragged_vertex_) {
                constraint_positions_.row(k) = new_pos_3d.cast<double>().transpose();
                break;
            }
            ++k;
        }

        solve_and_update();
        return true;
    };

    // --- Mouse Up: end drag ---
    viewer_.callback_mouse_up =
        [this](igl::opengl::glfw::Viewer& viewer, int button, int modifier) -> bool {
        if (!is_dragging_) return false;

        is_dragging_ = false;
        dragged_vertex_ = -1;
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

    int n = selected_vertices_.size();
    Eigen::VectorXi indices(n);
    constraint_positions_.resize(n, 3);

    int k = 0;
    for (int vid : selected_vertices_) {
        indices(k) = vid;
        constraint_positions_.row(k) = V_current_.row(vid);
        ++k;
    }

    deformer_.set_constraints(indices);
}

void ViewerApp::reset_mesh() {
    is_dragging_ = false;
    dragged_vertex_ = -1;
    V_current_ = V_;
    viewer_.data().set_vertices(V_current_);
    viewer_.data().compute_normals();
    int k = 0;
    for (int vid : selected_vertices_)
        constraint_positions_.row(k++) = V_.row(vid);
    update_overlay();
}

void ViewerApp::clear_selection() {
    is_dragging_ = false;
    dragged_vertex_ = -1;
    selected_vertices_.clear();
    constraint_positions_.resize(0, 3);
    sync_constraints();
    update_overlay();
}

void ViewerApp::solve_and_update() {
    if (!deformer_.is_prefactored()) return;

    auto t0 = std::chrono::high_resolution_clock::now();

    if (solver_mode_ == 1)
        V_current_ = deformer_.solve_arap(constraint_positions_, arap_iterations_);
    else
        V_current_ = deformer_.solve_laplacian(constraint_positions_);

    auto t1 = std::chrono::high_resolution_clock::now();
    last_solve_time_ms_ = std::chrono::duration<double, std::milli>(t1 - t0).count();

    viewer_.data().set_vertices(V_current_);
    viewer_.data().compute_normals();
    update_overlay();
}
