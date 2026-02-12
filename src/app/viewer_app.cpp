#include "viewer_app.h"
#include <igl/read_triangle_mesh.h>
#include <igl/project.h>
#include <igl/unproject.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/edges.h>
#include <igl/per_vertex_normals.h>
#include <iostream>
#include <chrono>
#include <algorithm>

namespace {
    constexpr double kSelectionRadius = 20.0;   // pixels
    constexpr double kOcclusionFraction = 0.3;  // fraction of mesh depth range
}

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

    // Precompute unique edge list
    igl::edges(F_, E_);

    return true;
}

void ViewerApp::launch() {
    viewer_.data().set_mesh(V_current_, F_);
    viewer_.data().compute_normals();
    viewer_.data().point_size = 10.0f;
    viewer_.data().line_width = 3.0f;
    viewer_.data().show_overlay_depth = true;

    // --- ImGui setup ---
    viewer_.plugins.push_back(&imgui_plugin_);
    imgui_plugin_.widgets.push_back(&imgui_menu_);

    imgui_menu_.callback_draw_viewer_window = []() {};  // hide default viewer window

    panel_.interaction_mode = &interaction_mode_;
    panel_.solver_mode = &solver_mode_;
    panel_.arap_iterations = &arap_iterations_;
    panel_.selection_element_mode = &selection_element_mode_;
    panel_.on_reset_mesh = [this]() { reset_mesh(); };
    panel_.on_clear_selection = [this]() { clear_selection(); };

    imgui_menu_.callback_draw_custom_window = [this]() {
        int prev_sel_mode = selection_element_mode_;
        panel_.num_vertices = V_.rows();
        panel_.num_faces = F_.rows();
        panel_.num_constraints = selected_vertices_.size();
        panel_.last_solve_time_ms = last_solve_time_ms_;
        panel_.draw();
        if (selection_element_mode_ != prev_sel_mode)
            update_overlay();
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
            case 'v': case 'V':
                selection_element_mode_ = 0;
                update_overlay();
                return true;
            case 'e': case 'E':
                selection_element_mode_ = 1;
                update_overlay();
                return true;
            case 'f': case 'F':
                selection_element_mode_ = 2;
                update_overlay();
                return true;
            default:
                return false;
        }
    };

    // --- Mouse Down: select mode or drag start ---
    viewer_.callback_mouse_down =
        [this](igl::opengl::glfw::Viewer& viewer, int button, int modifier) -> bool {
        if (button != 0) return false;  // left click only

        // Project all vertices to screen space
        Eigen::MatrixXd screen_verts;
        igl::project(V_current_, viewer.core().view, viewer.core().proj,
                     viewer.core().viewport, screen_verts);

        // Dynamic occlusion threshold based on mesh depth range
        double depth_range = screen_verts.col(2).maxCoeff() - screen_verts.col(2).minCoeff();
        double occlusion_eps = depth_range * kOcclusionFraction;

        // Viewport bounds for culling
        double vp_w = viewer.core().viewport(2);
        double vp_h = viewer.core().viewport(3);
        auto on_screen = [&](int i) {
            return screen_verts(i, 2) > 0.0 && screen_verts(i, 2) < 1.0
                && screen_verts(i, 0) >= 0.0 && screen_verts(i, 0) <= vp_w
                && screen_verts(i, 1) >= 0.0 && screen_verts(i, 1) <= vp_h;
        };

        // Find closest on-screen vertex to mouse click in screen space
        double mouse_x = static_cast<double>(viewer.current_mouse_x);
        double mouse_y = static_cast<double>(viewer.core().viewport(3) - viewer.current_mouse_y);

        // Occlusion: cast ray from mouse to get front surface depth
        double front_depth = 1.0;
        {
            int fid;
            Eigen::Vector3f bc;
            Eigen::Vector2f mouse_pos(viewer.current_mouse_x,
                                      viewer.core().viewport(3) - viewer.current_mouse_y);
            if (igl::unproject_onto_mesh(mouse_pos, viewer.core().view, viewer.core().proj,
                                         viewer.core().viewport, V_current_, F_, fid, bc)) {
                front_depth = bc(0) * screen_verts(F_(fid, 0), 2)
                            + bc(1) * screen_verts(F_(fid, 1), 2)
                            + bc(2) * screen_verts(F_(fid, 2), 2);
            }
        }

        double min_dist = std::numeric_limits<double>::max();
        int vid = -1;
        for (int i = 0; i < screen_verts.rows(); ++i) {
            if (!on_screen(i)) continue;
            double dx = screen_verts(i, 0) - mouse_x;
            double dy = screen_verts(i, 1) - mouse_y;
            double dist = dx * dx + dy * dy;
            if (dist < min_dist) {
                min_dist = dist;
                vid = i;
            }
        }

        if (interaction_mode_ == 0) {
            // Select mode: pick element based on selection_element_mode_
            if (selection_element_mode_ == 0) {
                // Vertex mode: find closest vertex
                if (vid < 0 || std::sqrt(min_dist) > kSelectionRadius) return false;
                if (screen_verts(vid, 2) > front_depth + occlusion_eps) return false;

                if (selected_vertices_.count(vid))
                    selected_vertices_.erase(vid);
                else
                    selected_vertices_.insert(vid);

            } else if (selection_element_mode_ == 1) {
                // Edge mode: find closest edge by point-to-segment distance
                double min_edge_dist = std::numeric_limits<double>::max();
                int best_edge = -1;
                for (int e = 0; e < E_.rows(); ++e) {
                    int i = E_(e, 0), j = E_(e, 1);
                    if (!on_screen(i) || !on_screen(j)) continue;
                    double p1x = screen_verts(i, 0), p1y = screen_verts(i, 1);
                    double p2x = screen_verts(j, 0), p2y = screen_verts(j, 1);
                    double dx = p2x - p1x, dy = p2y - p1y;
                    double len_sq = dx * dx + dy * dy;
                    double t = (len_sq > 0.0)
                        ? std::clamp(((mouse_x - p1x) * dx + (mouse_y - p1y) * dy) / len_sq, 0.0, 1.0)
                        : 0.0;
                    double cx = p1x + t * dx - mouse_x;
                    double cy = p1y + t * dy - mouse_y;
                    double dist = cx * cx + cy * cy;
                    if (dist < min_edge_dist) {
                        min_edge_dist = dist;
                        best_edge = e;
                    }
                }
                if (best_edge < 0 || std::sqrt(min_edge_dist) > kSelectionRadius) return false;

                int v0 = E_(best_edge, 0), v1 = E_(best_edge, 1);
                double edge_depth = std::min(screen_verts(v0, 2), screen_verts(v1, 2));
                if (edge_depth > front_depth + occlusion_eps) return false;
                bool all_selected = selected_vertices_.count(v0) && selected_vertices_.count(v1);
                if (all_selected) {
                    selected_vertices_.erase(v0);
                    selected_vertices_.erase(v1);
                } else {
                    selected_vertices_.insert(v0);
                    selected_vertices_.insert(v1);
                }

            } else {
                // Face mode: find closest face centroid
                double min_face_dist = std::numeric_limits<double>::max();
                int best_face = -1;
                for (int f = 0; f < F_.rows(); ++f) {
                    if (!on_screen(F_(f,0)) || !on_screen(F_(f,1)) || !on_screen(F_(f,2))) continue;
                    double cx = (screen_verts(F_(f,0), 0) + screen_verts(F_(f,1), 0) + screen_verts(F_(f,2), 0)) / 3.0;
                    double cy = (screen_verts(F_(f,0), 1) + screen_verts(F_(f,1), 1) + screen_verts(F_(f,2), 1)) / 3.0;
                    double dx = cx - mouse_x, dy = cy - mouse_y;
                    double dist = dx * dx + dy * dy;
                    if (dist < min_face_dist) {
                        min_face_dist = dist;
                        best_face = f;
                    }
                }
                if (best_face < 0 || std::sqrt(min_face_dist) > kSelectionRadius) return false;

                int v0 = F_(best_face, 0), v1 = F_(best_face, 1), v2 = F_(best_face, 2);
                double face_depth = std::min({screen_verts(v0, 2), screen_verts(v1, 2), screen_verts(v2, 2)});
                if (face_depth > front_depth + occlusion_eps) return false;
                bool all_selected = selected_vertices_.count(v0)
                                 && selected_vertices_.count(v1)
                                 && selected_vertices_.count(v2);
                if (all_selected) {
                    selected_vertices_.erase(v0);
                    selected_vertices_.erase(v1);
                    selected_vertices_.erase(v2);
                } else {
                    selected_vertices_.insert(v0);
                    selected_vertices_.insert(v1);
                    selected_vertices_.insert(v2);
                }
            }

            sync_constraints();
            update_overlay();
            return true;
        } else {
            // Drag mode: start drag if nearest vertex is selected
            if (vid < 0 || std::sqrt(min_dist) > kSelectionRadius) return false;
            if (screen_verts(vid, 2) > front_depth + occlusion_eps) return false;
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

    update_overlay();
    viewer_.launch();
}

void ViewerApp::update_overlay() {
    viewer_.data().clear_points();
    viewer_.data().clear_edges();

    const Eigen::RowVector3d red(1.0, 0.0, 0.0);
    const Eigen::RowVector3d green(0.0, 1.0, 0.0);
    const Eigen::RowVector3d orange(1.0, 0.6, 0.0);

    // Offset overlay positions slightly along vertex normals to avoid z-fighting
    Eigen::MatrixXd N;
    igl::per_vertex_normals(V_current_, F_, N);
    double diag = (V_current_.colwise().maxCoeff() - V_current_.colwise().minCoeff()).norm();
    Eigen::MatrixXd V_overlay = V_current_ + N * (diag * 0.002);

    // Reset per-face colors when not in face mode
    if (selection_element_mode_ != 2) {
        Eigen::MatrixXd C(1, 3);
        C.row(0) = Eigen::RowVector3d(0.85, 0.85, 0.85);
        viewer_.data().set_colors(C);
    }

    if (selection_element_mode_ == 0) {
        // Vertex mode: unselected as orange, selected as red
        for (int i = 0; i < V_overlay.rows(); ++i) {
            if (selected_vertices_.count(i)) {
                Eigen::RowVector3d color = (i == dragged_vertex_) ? green : red;
                viewer_.data().add_points(V_overlay.row(i), color);
            } else {
                viewer_.data().add_points(V_overlay.row(i), orange);
            }
        }

    } else if (selection_element_mode_ == 1) {
        // Edge mode: batch all edges into P1, P2, C matrices
        int ne = E_.rows();
        Eigen::MatrixXd P1(ne, 3), P2(ne, 3), C(ne, 3);
        for (int e = 0; e < ne; ++e) {
            int i = E_(e, 0), j = E_(e, 1);
            P1.row(e) = V_overlay.row(i);
            P2.row(e) = V_overlay.row(j);
            bool selected = selected_vertices_.count(i) && selected_vertices_.count(j);
            C.row(e) = selected ? red : orange;
        }
        viewer_.data().add_edges(P1, P2, C);

        // Draw selected vertices as points
        for (int vid : selected_vertices_) {
            Eigen::RowVector3d color = (vid == dragged_vertex_) ? green : red;
            viewer_.data().add_points(V_overlay.row(vid), color);
        }

    } else {
        // Face mode: all faces orange, selected faces red
        Eigen::MatrixXd C(F_.rows(), 3);
        for (int f = 0; f < F_.rows(); ++f) {
            bool all_selected = selected_vertices_.count(F_(f, 0))
                             && selected_vertices_.count(F_(f, 1))
                             && selected_vertices_.count(F_(f, 2));
            C.row(f) = all_selected ? red : orange;
        }
        viewer_.data().set_colors(C);

        // Draw selected vertices as points
        for (int vid : selected_vertices_) {
            Eigen::RowVector3d color = (vid == dragged_vertex_) ? green : red;
            viewer_.data().add_points(V_overlay.row(vid), color);
        }
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
