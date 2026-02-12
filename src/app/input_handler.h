#pragma once

#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/Core>
#include <set>
#include <functional>

class InputHandler {
public:
    // Shared state (set by ViewerApp before bind)
    const Eigen::MatrixXd* V_current = nullptr;
    const Eigen::MatrixXi* F = nullptr;
    const Eigen::MatrixXi* E = nullptr;
    std::set<int>* selected_vertices = nullptr;
    Eigen::MatrixXd* constraint_positions = nullptr;

    int* interaction_mode = nullptr;       // 0=Select, 1=Drag
    int* solver_mode = nullptr;            // 0=Laplacian, 1=ARAP
    int* selection_element_mode = nullptr; // 0=Vertex, 1=Edge, 2=Face
    int* arap_iterations = nullptr;

    int dragged_vertex = -1;
    bool is_dragging = false;
    float drag_depth = 0.0f;

    // Callbacks (set by ViewerApp)
    std::function<void()> on_selection_changed;
    std::function<void()> on_drag_moved;
    std::function<void()> on_overlay_needs_update;

    // Register all callbacks on the viewer
    void bind(igl::opengl::glfw::Viewer& viewer);

    bool on_key_pressed(igl::opengl::glfw::Viewer& viewer, unsigned int key, int modifier);

private:
    bool on_mouse_down(igl::opengl::glfw::Viewer& viewer, int button, int modifier);
    bool on_mouse_move(igl::opengl::glfw::Viewer& viewer, int mouse_x, int mouse_y);
    bool on_mouse_up(igl::opengl::glfw::Viewer& viewer, int button, int modifier);

    // Picking helpers
    struct PickContext {
        Eigen::MatrixXd screen_verts;
        double mouse_x, mouse_y;
        double front_depth;
        double occlusion_eps;
        std::function<bool(int)> on_screen;
    };

    PickContext build_pick_context(igl::opengl::glfw::Viewer& viewer);
    int pick_nearest_vertex(const PickContext& ctx);
    int pick_edge(const PickContext& ctx);
    int pick_face(const PickContext& ctx);
    void toggle_vertices(const std::vector<int>& vids);
};
