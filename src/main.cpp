#include "viewer_app.h"
#include <iostream>
#include <string>

int main(int argc, char *argv[]) {
    std::string mesh_path;
    if (argc > 1) {
        mesh_path = argv[1];
    } else {
        mesh_path = DATA_DIR "/spot.obj";
        std::cout << "Usage: " << argv[0] << " <mesh.(obj|off|ply|stl)>" << std::endl;
        std::cout << "No mesh specified, loading default: " << mesh_path << std::endl;
    }

    ViewerApp app;
    if (!app.load_mesh(mesh_path)) {
        return 1;
    }

    app.launch();
    return 0;
}
