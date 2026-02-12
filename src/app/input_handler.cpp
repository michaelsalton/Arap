#include "input_handler.h"
#include <igl/project.h>
#include <igl/unproject.h>
#include <igl/unproject_onto_mesh.h>
#include <algorithm>
#include <cmath>

namespace {
    constexpr double kSelectionRadius = 20.0;
    constexpr double kOcclusionFraction = 0.3;
}

void InputHandler::bind(igl::opengl::glfw::Viewer& viewer) {
    viewer.callback_key_pressed =
        [this](igl::opengl::glfw::Viewer& v, unsigned int key, int mod) {
        return on_key_pressed(v, key, mod);
    };
    viewer.callback_mouse_down =
        [this](igl::opengl::glfw::Viewer& v, int button, int mod) {
        return on_mouse_down(v, button, mod);
    };
    viewer.callback_mouse_move =
        [this](igl::opengl::glfw::Viewer& v, int mx, int my) {
        return on_mouse_move(v, mx, my);
    };
    viewer.callback_mouse_up =
        [this](igl::opengl::glfw::Viewer& v, int button, int mod) {
        return on_mouse_up(v, button, mod);
    };
}

bool InputHandler::on_key_pressed(igl::opengl::glfw::Viewer&, unsigned int key, int) {
    switch (key) {
        case ' ':
            *interaction_mode = 1 - *interaction_mode;
            return true;
        case 'r': case 'R':
            // Reset is handled by ViewerApp via on_selection_changed after clear
            return false;
        case '1':
            *solver_mode = 0;
            return true;
        case '2':
            *solver_mode = 1;
            return true;
        case 'v': case 'V':
            *selection_element_mode = 0;
            if (on_overlay_needs_update) on_overlay_needs_update();
            return true;
        case 'e': case 'E':
            *selection_element_mode = 1;
            if (on_overlay_needs_update) on_overlay_needs_update();
            return true;
        case 'f': case 'F':
            *selection_element_mode = 2;
            if (on_overlay_needs_update) on_overlay_needs_update();
            return true;
        default:
            return false;
    }
}

InputHandler::PickContext InputHandler::build_pick_context(igl::opengl::glfw::Viewer& viewer) {
    PickContext ctx;

    igl::project(*V_current, viewer.core().view, viewer.core().proj,
                 viewer.core().viewport, ctx.screen_verts);

    double depth_range = ctx.screen_verts.col(2).maxCoeff() - ctx.screen_verts.col(2).minCoeff();
    ctx.occlusion_eps = depth_range * kOcclusionFraction;

    double vp_w = viewer.core().viewport(2);
    double vp_h = viewer.core().viewport(3);
    ctx.on_screen = [&ctx, vp_w, vp_h](int i) {
        return ctx.screen_verts(i, 2) > 0.0 && ctx.screen_verts(i, 2) < 1.0
            && ctx.screen_verts(i, 0) >= 0.0 && ctx.screen_verts(i, 0) <= vp_w
            && ctx.screen_verts(i, 1) >= 0.0 && ctx.screen_verts(i, 1) <= vp_h;
    };

    ctx.mouse_x = static_cast<double>(viewer.current_mouse_x);
    ctx.mouse_y = static_cast<double>(viewer.core().viewport(3) - viewer.current_mouse_y);

    ctx.front_depth = 1.0;
    {
        int fid;
        Eigen::Vector3f bc;
        Eigen::Vector2f mouse_pos(viewer.current_mouse_x,
                                  viewer.core().viewport(3) - viewer.current_mouse_y);
        if (igl::unproject_onto_mesh(mouse_pos, viewer.core().view, viewer.core().proj,
                                     viewer.core().viewport, *V_current, *F, fid, bc)) {
            ctx.front_depth = bc(0) * ctx.screen_verts((*F)(fid, 0), 2)
                            + bc(1) * ctx.screen_verts((*F)(fid, 1), 2)
                            + bc(2) * ctx.screen_verts((*F)(fid, 2), 2);
        }
    }

    return ctx;
}

int InputHandler::pick_nearest_vertex(const PickContext& ctx) {
    double min_dist = std::numeric_limits<double>::max();
    int vid = -1;
    for (int i = 0; i < ctx.screen_verts.rows(); ++i) {
        if (!ctx.on_screen(i)) continue;
        double dx = ctx.screen_verts(i, 0) - ctx.mouse_x;
        double dy = ctx.screen_verts(i, 1) - ctx.mouse_y;
        double dist = dx * dx + dy * dy;
        if (dist < min_dist) {
            min_dist = dist;
            vid = i;
        }
    }
    if (vid < 0 || std::sqrt(min_dist) > kSelectionRadius) return -1;
    if (ctx.screen_verts(vid, 2) > ctx.front_depth + ctx.occlusion_eps) return -1;
    return vid;
}

int InputHandler::pick_edge(const PickContext& ctx) {
    double min_edge_dist = std::numeric_limits<double>::max();
    int best_edge = -1;
    for (int e = 0; e < E->rows(); ++e) {
        int i = (*E)(e, 0), j = (*E)(e, 1);
        if (!ctx.on_screen(i) || !ctx.on_screen(j)) continue;
        double p1x = ctx.screen_verts(i, 0), p1y = ctx.screen_verts(i, 1);
        double p2x = ctx.screen_verts(j, 0), p2y = ctx.screen_verts(j, 1);
        double dx = p2x - p1x, dy = p2y - p1y;
        double len_sq = dx * dx + dy * dy;
        double t = (len_sq > 0.0)
            ? std::clamp(((ctx.mouse_x - p1x) * dx + (ctx.mouse_y - p1y) * dy) / len_sq, 0.0, 1.0)
            : 0.0;
        double cx = p1x + t * dx - ctx.mouse_x;
        double cy = p1y + t * dy - ctx.mouse_y;
        double dist = cx * cx + cy * cy;
        if (dist < min_edge_dist) {
            min_edge_dist = dist;
            best_edge = e;
        }
    }
    if (best_edge < 0 || std::sqrt(min_edge_dist) > kSelectionRadius) return -1;

    int v0 = (*E)(best_edge, 0), v1 = (*E)(best_edge, 1);
    double edge_depth = std::min(ctx.screen_verts(v0, 2), ctx.screen_verts(v1, 2));
    if (edge_depth > ctx.front_depth + ctx.occlusion_eps) return -1;
    return best_edge;
}

int InputHandler::pick_face(const PickContext& ctx) {
    double min_face_dist = std::numeric_limits<double>::max();
    int best_face = -1;
    for (int f = 0; f < F->rows(); ++f) {
        if (!ctx.on_screen((*F)(f,0)) || !ctx.on_screen((*F)(f,1)) || !ctx.on_screen((*F)(f,2))) continue;
        double cx = (ctx.screen_verts((*F)(f,0), 0) + ctx.screen_verts((*F)(f,1), 0) + ctx.screen_verts((*F)(f,2), 0)) / 3.0;
        double cy = (ctx.screen_verts((*F)(f,0), 1) + ctx.screen_verts((*F)(f,1), 1) + ctx.screen_verts((*F)(f,2), 1)) / 3.0;
        double dx = cx - ctx.mouse_x, dy = cy - ctx.mouse_y;
        double dist = dx * dx + dy * dy;
        if (dist < min_face_dist) {
            min_face_dist = dist;
            best_face = f;
        }
    }
    if (best_face < 0 || std::sqrt(min_face_dist) > kSelectionRadius) return -1;

    int v0 = (*F)(best_face, 0), v1 = (*F)(best_face, 1), v2 = (*F)(best_face, 2);
    double face_depth = std::min({ctx.screen_verts(v0, 2), ctx.screen_verts(v1, 2), ctx.screen_verts(v2, 2)});
    if (face_depth > ctx.front_depth + ctx.occlusion_eps) return -1;
    return best_face;
}

void InputHandler::toggle_vertices(const std::vector<int>& vids) {
    bool all_selected = true;
    for (int v : vids) {
        if (!selected_vertices->count(v)) { all_selected = false; break; }
    }
    if (all_selected) {
        for (int v : vids) selected_vertices->erase(v);
    } else {
        for (int v : vids) selected_vertices->insert(v);
    }
}

bool InputHandler::on_mouse_down(igl::opengl::glfw::Viewer& viewer, int button, int) {
    if (button != 0) return false;

    PickContext ctx = build_pick_context(viewer);

    if (*interaction_mode == 0) {
        // Select mode
        if (*selection_element_mode == 0) {
            int vid = pick_nearest_vertex(ctx);
            if (vid < 0) return false;
            toggle_vertices({vid});
        } else if (*selection_element_mode == 1) {
            int eid = pick_edge(ctx);
            if (eid < 0) return false;
            toggle_vertices({(*E)(eid, 0), (*E)(eid, 1)});
        } else {
            int fid = pick_face(ctx);
            if (fid < 0) return false;
            toggle_vertices({(*F)(fid, 0), (*F)(fid, 1), (*F)(fid, 2)});
        }

        if (on_selection_changed) on_selection_changed();
        return true;
    } else {
        // Drag mode
        int vid = pick_nearest_vertex(ctx);
        if (vid < 0 || !selected_vertices->count(vid)) return false;

        is_dragging = true;
        dragged_vertex = vid;

        Eigen::Vector3f screen_pos = igl::project(
            Eigen::Vector3f(V_current->row(vid).cast<float>()),
            viewer.core().view, viewer.core().proj, viewer.core().viewport);
        drag_depth = screen_pos(2);

        if (on_overlay_needs_update) on_overlay_needs_update();
        return true;
    }
}

bool InputHandler::on_mouse_move(igl::opengl::glfw::Viewer& viewer, int mouse_x, int mouse_y) {
    if (!is_dragging) return false;

    float screen_y = viewer.core().viewport(3) - static_cast<float>(mouse_y);
    Eigen::Vector3f new_pos_3d = igl::unproject(
        Eigen::Vector3f(static_cast<float>(mouse_x), screen_y, drag_depth),
        viewer.core().view, viewer.core().proj, viewer.core().viewport);

    int k = 0;
    for (int vid : *selected_vertices) {
        if (vid == dragged_vertex) {
            constraint_positions->row(k) = new_pos_3d.cast<double>().transpose();
            break;
        }
        ++k;
    }

    if (on_drag_moved) on_drag_moved();
    return true;
}

bool InputHandler::on_mouse_up(igl::opengl::glfw::Viewer&, int, int) {
    if (!is_dragging) return false;

    is_dragging = false;
    dragged_vertex = -1;
    if (on_overlay_needs_update) on_overlay_needs_update();
    return true;
}
