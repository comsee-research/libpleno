#include "gui.h"

bool Viewer::_enable = true;

v::ViewerContext Viewer::_viewer[2] = {
	v::viewer(0).title("2D view").size(800, 600).background(v::dark_gray),
	v::viewer(1).title("3D view").size(800, 600).background(v::dark_gray).interaction_mode(v::INTERACTION_CAD)
};

Viewer::Layer Viewer::_layer[2] = {0,0};
Viewer::Layer Viewer::_saved_layer[2] = {0,0};
