#include "demo_postfx.h"

#include <stereokit.h>
#include <stereokit_ui.h>

#include "skt_postfx_vignette.hlsl.h"
#include "skt_postfx_invert.hlsl.h"
#include "skt_postfx_depth_fog.hlsl.h"

using namespace sk;

///////////////////////////////////////////

static shader_t   postfx_vignette_shader = {};
static shader_t   postfx_invert_shader   = {};
static shader_t   postfx_fog_shader      = {};
static material_t postfx_vignette        = {};
static material_t postfx_invert          = {};
static material_t postfx_fog             = {};
static model_t    postfx_model           = {};
static bool32_t   vignette_on            = false;
static bool32_t   invert_on              = false;
static bool32_t   fog_on                 = false;
static float      vignette_strength      = 0.4f;
static float      fog_density            = 0.5f;

///////////////////////////////////////////

void demo_postfx_init() {
	postfx_vignette_shader = shader_create_mem((void*)sks_skt_postfx_vignette_hlsl,  sizeof(sks_skt_postfx_vignette_hlsl));
	postfx_invert_shader   = shader_create_mem((void*)sks_skt_postfx_invert_hlsl,    sizeof(sks_skt_postfx_invert_hlsl));
	postfx_fog_shader      = shader_create_mem((void*)sks_skt_postfx_depth_fog_hlsl, sizeof(sks_skt_postfx_depth_fog_hlsl));
	postfx_vignette        = material_create(postfx_vignette_shader);
	postfx_invert          = material_create(postfx_invert_shader);
	postfx_fog             = material_create(postfx_fog_shader);

	// Queue offsets order the chain - fog reads scene depth first, then the
	// invert flips colors, and the vignette darkens the corners last.
	material_set_queue_offset(postfx_fog,     -10);
	material_set_queue_offset(postfx_invert,    0);
	material_set_queue_offset(postfx_vignette, 10);

	postfx_model = model_create_file("DamagedHelmet.gltf");
}

///////////////////////////////////////////

void demo_postfx_update() {
	// Helmet on the left, settings on the right, centered together in front.
	model_draw(postfx_model, matrix_trs(vec3{-0.175f,-0.1f,-0.5f}, quat_from_angles(0,200,0), vec3{0.175f,0.175f,0.175f}));

	static pose_t window_pose =
		pose_t{ {0.175f,-0.05f,-0.4f}, quat_lookat({0.175f,-0.05f,-0.4f}, {0.175f,-0.05f,0}) };

	ui_window_begin("PostFX", &window_pose);

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

	if (ui_toggle("Depth fog", fog_on)) {
		if (fog_on) render_add_post_process   (postfx_fog);
		else        render_remove_post_process(postfx_fog);
	}
	if (fog_on && ui_hslider("density", fog_density, 0, 2))
		material_set_float(postfx_fog, "fog_density", fog_density);

	ui_window_end();
}

///////////////////////////////////////////

void demo_postfx_shutdown() {
	render_remove_post_process(postfx_vignette);
	render_remove_post_process(postfx_invert);
	render_remove_post_process(postfx_fog);
	vignette_on = false;
	invert_on   = false;
	fog_on      = false;

	material_release(postfx_vignette);
	material_release(postfx_invert);
	material_release(postfx_fog);
	shader_release  (postfx_vignette_shader);
	shader_release  (postfx_invert_shader);
	shader_release  (postfx_fog_shader);
	model_release   (postfx_model);
}
