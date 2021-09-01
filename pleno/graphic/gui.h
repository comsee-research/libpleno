#pragma once

//LIBV
#include <libv/core/conversions/opencv.hpp>
#include <libv/core/image/image.hpp>

#include <libv/graphic/viewer_context.hpp>
#include <libv/graphic/colors.hpp>
#include <libv/graphic/palette.hpp>

#include "draw.h"

#include "io/printer.h"

namespace tag {
	struct CameraBody {};
	struct ThinLens {};
	struct MLA {};
	struct Sensor {};	
	struct Barycenters {};
};

class Viewer {
public:
	using Layer = unsigned int;
	enum Mode : bool { m2D=0, m3D =1 };
private:
	static bool _enable;

	static v::ViewerContext _viewer[2];

	static Layer _layer[2];
	static Layer _saved_layer[2];

	Viewer() noexcept {}
public:
	Viewer(const Viewer&) = delete;
	Viewer& operator=(const Viewer&) = delete;

	Viewer(Viewer&&) = delete;
	Viewer&& operator=(Viewer&&) = delete;

	static v::ViewerContext& context(Mode m = m2D) { return _viewer[m]; }

	static Layer& layer(Mode m = m2D) { return _layer[m]; }

	static bool enable() { return _enable; } 
	static void enable(bool e) { _enable=e; }

	static void stash(Mode m = m2D) { _saved_layer[m] = _layer[m]; }
	static void pop(Mode m = m2D) { _layer[m] = _saved_layer[m]; }

	static void update(Mode m = m2D) { Viewer::context(m).layer(Viewer::layer(m)++).update().clear();}
	static void clear(Mode m = m2D) {for(Layer l=0; l < _layer[m]; ++l) Viewer::context(m).layer(l).update().clear(); _layer[m] = 0;}
	static void clear_all() {clear(Mode::m2D); clear(Mode::m3D);}

	static void wait() { v::wait_viewers(); }
	static void stop() { v::stop_viewers(); }
};


#define RENDER_DEBUG_2D(...) do { if(Viewer::enable()) { viewer_2d(__VA_ARGS__); } } while(0)
#define RENDER_DEBUG_3D(...) do { if(Viewer::enable()) { viewer_3d(__VA_ARGS__); } } while(0)
#define GUI(...) do { if(Viewer::enable()) { __VA_ARGS__ } } while(0)

const auto FORCE_GUI = [](bool enable) -> void { 
	static bool gui_state = Viewer::enable();
	
	if(enable and not(gui_state)) {
		PRINT_INFO("Enabling GUI...");
		gui_state = Viewer::enable();
		Viewer::enable(true);
	}
	
	if(not enable) Viewer::enable(gui_state);
};

