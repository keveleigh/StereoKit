/* SPDX-License-Identifier: MIT */
/* The authors below grant copyright rights under the MIT license:
 * Copyright (c) 2024 Nick Klingensmith
 * Copyright (c) 2024 Qualcomm Technologies, Inc.
 */

#include "../stereokit.h"
#include "input.h"
#include "input_render.h"
#include "../hands/input_hand.h"
#include "../hands/hand_oxr_articulated.h"
#include "../libraries/array.h"
#include "../xr_backends/openxr.h"
#include "../xr_backends/openxr_extensions.h"
#include "../systems/defaults.h"
#include "../asset_types/model.h"

#include <string>

namespace sk {

///////////////////////////////////////////

struct input_event_t {
	input_source_ source;
	button_state_ event;
	void (*event_callback)(input_source_ source, button_state_ evt, const pointer_t &pointer);
};

struct input_render_state_t {
	input_render_mode_ render_mode;
	hand_mesh_t        hand_fallback_mesh   [2];
	hand_mesh_t        hand_articulated_mesh[2];
	material_t         hand_material        [2];
	model_t            controller_model     [2];
	bool               model_is_fallback    = false;
};
static input_render_state_t local = {};

///////////////////////////////////////////

void input_hand_update_fallback_mesh(handed_ handed, hand_mesh_t* hand_mesh);

///////////////////////////////////////////

bool input_render_init() {
	local = {};
	local.render_mode = input_render_hand_fallback;

	input_controller_model_set(handed_left,  nullptr); // Assigning to null sets it to default
	input_controller_model_set(handed_right, nullptr);

	material_t hand_mat = material_copy_id(default_id_material);
	material_set_id          (hand_mat, default_id_material_hand);
	material_set_transparency(hand_mat, transparency_blend);

	gradient_t color_grad = gradient_create();
	gradient_add(color_grad, color128{ .4f,.4f,.4f,0 }, 0.0f);
	gradient_add(color_grad, color128{ .6f,.6f,.6f,0 }, 0.4f);
	gradient_add(color_grad, color128{ .8f,.8f,.8f,1 }, 0.55f);
	gradient_add(color_grad, color128{ 1,  1,  1,  1 }, 1.0f);

	color32 gradient[16 * 16];
	for (int32_t y = 0; y < 16; y++) {
		color32 col = gradient_get32(color_grad, 1-y/15.f);
		for (int32_t x = 0; x < 16; x++) {
			gradient[x + y * 16] = col;
		}
	}
	gradient_destroy(color_grad);

	tex_t gradient_tex = tex_create(tex_type_image, tex_format_rgba32_linear);
	tex_set_colors (gradient_tex, 16, 16, gradient);
	tex_set_address(gradient_tex, tex_address_clamp);
	material_set_texture     (hand_mat, "diffuse", gradient_tex);
	material_set_queue_offset(hand_mat, 10);

	input_hand_material(handed_max, hand_mat);

	tex_release(gradient_tex);
	material_release(hand_mat);

	for (int32_t i = 0; i < handed_max; i++) {
		// Set up the hand mesh
		local.hand_fallback_mesh[i].root_transform = matrix_identity;
		local.hand_fallback_mesh[i].mesh           = mesh_create();
		mesh_set_keep_data(local.hand_fallback_mesh[i].mesh, false);
		mesh_set_id       (local.hand_fallback_mesh[i].mesh, i == handed_left
			? default_id_mesh_lefthand
			: default_id_mesh_righthand);
	}

	return true;
}

///////////////////////////////////////////

void input_render_shutdown() {
	for (int32_t i = 0; i < handed_max; i++) {
		model_release(local.controller_model[i]);
		material_release(local.hand_material[i]);
		mesh_release    (local.hand_fallback_mesh[i].mesh);
		sk_free(local.hand_fallback_mesh[i].inds);
		sk_free(local.hand_fallback_mesh[i].verts);
		mesh_release(local.hand_articulated_mesh[i].mesh);
		sk_free(local.hand_articulated_mesh[i].inds);
		sk_free(local.hand_articulated_mesh[i].verts);
	}
	local = {};
}

///////////////////////////////////////////

void input_render_step() {
}

///////////////////////////////////////////

void input_render_step_late() {
	// Don't draw the input if the app isn't in focus, this is a Quest store
	// requirement! It generally seems like the correct choice when an overlay
	// may be overtop the app, rendering input of its own.
	if (sk_app_focus() != app_focus_active) return;

	for (int32_t i = 0; i < handed_max; i++) {
		if (input_hand_get_visible((handed_)i) == false) continue;

		hand_source_ source = input_hand_source((handed_)i);
		if (source == hand_source_articulated) {
			const hand_t* hand = input_hand((handed_)i);
			if ((hand->tracked_state & button_state_active) != 0 && local.hand_material[i] != nullptr) {
				hand_mesh_t* hand_active_mesh = &local.hand_fallback_mesh[i];
				if (xr_ext.MSFT_hand_tracking_mesh == xr_ext_active) { hand_oxra_update_system_mesh   ((handed_)i, &local.hand_articulated_mesh  [i]); hand_active_mesh = &local.hand_articulated_mesh[i]; }
				else                                                 { input_hand_update_fallback_mesh((handed_)i, &local.hand_fallback_mesh     [i]); hand_active_mesh = &local.hand_fallback_mesh   [i]; }

				render_add_mesh(hand_active_mesh->mesh, local.hand_material[i], hand_active_mesh->root_transform, hand->pinch_state & button_state_active ? color128{ 1.5f, 1.5f, 1.5f, 1 } : color128{ 1,1,1,1 });
			}
		} else if (source == hand_source_simulated) {
			const controller_t* control = input_controller((handed_)i);
			if ((control->tracked & button_state_active) != 0 && local.controller_model[i] != nullptr) {
				// If we're using a fallback model from the system, it may need updating
				if (local.model_is_fallback) {
					input_controller_model_set((handed_)i, nullptr);
				}
				render_add_model(local.controller_model[i], matrix_trs(control->pose.position, control->pose.orientation));
			}
		} else if (source == hand_source_overridden) {
			const hand_t* hand = input_hand((handed_)i);
			if ((hand->tracked_state & button_state_active) != 0 && local.hand_material[i] != nullptr) {
				input_hand_update_fallback_mesh((handed_)i, &local.hand_fallback_mesh[i]);

				render_add_mesh(local.hand_fallback_mesh[i].mesh, local.hand_material[i], local.hand_fallback_mesh[i].root_transform, hand->pinch_state & button_state_active ? color128{ 1.5f, 1.5f, 1.5f, 1 } : color128{ 1,1,1,1 });
			}
		}
	}
}

///////////////////////////////////////////

void input_hand_update_fallback_mesh(handed_ handed, hand_mesh_t* hand_mesh) {
	input_gen_fallback_mesh(input_hand(handed)->fingers, hand_mesh->mesh, &hand_mesh->verts, &hand_mesh->inds);
}

///////////////////////////////////////////

void input_hand_material(handed_ hand, material_t material) {
	if (hand == handed_max) {
		input_hand_material(handed_left,  material);
		input_hand_material(handed_right, material);
		return;
	}

	if (material != nullptr)
		material_addref(material);
	material_release(local.hand_material[hand]);

	local.hand_material[hand] = material;
}

///////////////////////////////////////////

void input_controller_model_set(handed_ hand, model_t model) {
	if (hand == handed_max) {
		input_controller_model_set(handed_left,  model);
		input_controller_model_set(handed_right, model);
		return;
	}

	// If the model is null, check if a model is available via XR_MSFT_controller_model.
	// Otherwise, set it to the default controller model.
	if (model == nullptr) {
		if (xr_ext.MSFT_controller_model == xr_ext_active) {
			XrPath hand_path;
			xrStringToPath(xr_instance, hand == handed_left ? "/user/hand/left" : "/user/hand/right", &hand_path);
			XrControllerModelKeyStateMSFT key_state = {XR_TYPE_CONTROLLER_MODEL_KEY_STATE_MSFT};
			if (XR_SUCCEEDED(xr_extensions.xrGetControllerModelKeyMSFT(xr_session, hand_path, &key_state)) &&
				key_state.modelKey != XR_NULL_CONTROLLER_MODEL_KEY_MSFT) {
				std::string key_str = std::to_string(key_state.modelKey);
				model = model_find(key_str.c_str());
				if (model == nullptr)
				{
					uint32_t buffer_capacity_input;
					uint32_t buffer_count_output;
					if (XR_SUCCEEDED(xr_extensions.xrLoadControllerModelMSFT(xr_session, key_state.modelKey, 0, &buffer_capacity_input, nullptr))) {
						array_t<uint8_t> model_buffer = array_t<uint8_t>::make(buffer_capacity_input);
						if (XR_SUCCEEDED(xr_extensions.xrLoadControllerModelMSFT(xr_session, key_state.modelKey, buffer_capacity_input, &buffer_count_output, model_buffer.data))) {
							model = model_create_mem(
								hand == handed_left ? ("msft/controller_l_" + key_str + ".glb").c_str() : ("msft/controller_r_" + key_str + ".glb").c_str(),
								model_buffer.data, buffer_count_output, sk_default_shader);
							// Models need to be rotated 180° to align with the user holding them
							sk::model_node_id root_node = model_node_get_root(model);
							model_node_set_transform_local(model, root_node, model_node_get_transform_local(model, root_node) * matrix_from_angles(0, 180, 0));
							model_set_id(model, key_str.c_str());
							model->nodes_used = buffer_capacity_input;
						}
					}
				}
				else {
					//array_t<XrControllerModelNodePropertiesMSFT> node_properties = array_t<XrControllerModelNodePropertiesMSFT>::make(buffer_capacity_input);
					//XrControllerModelPropertiesMSFT model_properties = {XR_TYPE_CONTROLLER_MODEL_PROPERTIES_MSFT};
					//model_properties.nodeCapacityInput = buffer_capacity_input;
					//model_properties.nodeProperties = node_properties.data;
					//xr_extensions.xrGetControllerModelPropertiesMSFT(xr_session, key_state.modelKey, &model_properties);
				}
			}

			if (model == nullptr) {
				model = hand == handed_left ? sk_default_controller_l : sk_default_controller_r;
			}
		}
		else {
			model = hand == handed_left ? sk_default_controller_l : sk_default_controller_r;
		}

		local.model_is_fallback = true;
	}
	else {
		local.model_is_fallback = false;
	}

	if (model != local.controller_model[hand]) {
		model_addref(model);
		model_release(local.controller_model[hand]);

		local.controller_model[hand] = model;
	}
}

///////////////////////////////////////////

model_t input_controller_model_get(handed_ hand) {
	if (local.controller_model[hand] != nullptr)
		model_addref(local.controller_model[hand]);
	return local.controller_model[hand];
}

} // namespace sk