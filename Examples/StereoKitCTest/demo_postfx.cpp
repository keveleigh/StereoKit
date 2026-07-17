#include "demo_postfx.h"

#include <stereokit.h>
#include <stereokit_ui.h>

#include "skt_postfx_vignette.hlsl.h"
#include "skt_postfx_invert.hlsl.h"

using namespace sk;

///////////////////////////////////////////

static shader_t   postfx_vignette_shader = {};
static shader_t   postfx_invert_shader   = {};
static material_t postfx_vignette        = {};
static material_t postfx_invert          = {};
static bool32_t   vignette_on            = false;
static bool32_t   invert_on              = false;
static float      vignette_strength      = 0.4f;

///////////////////////////////////////////

void demo_postfx_init() {
	postfx_vignette_shader = shader_create_mem((void*)sks_skt_postfx_vignette_hlsl, sizeof(sks_skt_postfx_vignette_hlsl));
	postfx_invert_shader   = shader_create_mem((void*)sks_skt_postfx_invert_hlsl,   sizeof(sks_skt_postfx_invert_hlsl));
	postfx_vignette        = material_create(postfx_vignette_shader);
	postfx_invert          = material_create(postfx_invert_shader);

	// Queue offsets order the chain - the vignette darkens corners after the
	// invert has flipped them.
	material_set_queue_offset(postfx_invert,   0);
	material_set_queue_offset(postfx_vignette, 10);
}

///////////////////////////////////////////

void demo_postfx_update() {
	static pose_t window_pose =
		pose_t{ {0.25f,0.25f,-0.25f}, quat_lookat({0.25f,0.25f,-0.25f}, {0,0.25f,0}) };

	ui_window_begin("Post Processing", &window_pose);

	if (ui_toggle("Vignette", vignette_on)) {
		if (vignette_on) render_add_post_process   (postfx_vignette);
		else             render_remove_post_process(postfx_vignette);
	}
	if (vignette_on && ui_hslider("strength", vignette_strength, 0, 1))
		material_set_float(postfx_vignette, "strength", vignette_strength);

	if (ui_toggle("Invert", invert_on)) {
		if (invert_on) render_add_post_process   (postfx_invert);
		else           render_remove_post_process(postfx_invert);
	}

	ui_window_end();
}

///////////////////////////////////////////

void demo_postfx_shutdown() {
	render_remove_post_process(postfx_vignette);
	render_remove_post_process(postfx_invert);
	vignette_on = false;
	invert_on   = false;

	material_release(postfx_vignette);
	material_release(postfx_invert);
	shader_release  (postfx_vignette_shader);
	shader_release  (postfx_invert_shader);
}
