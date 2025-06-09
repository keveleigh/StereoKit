/* SPDX-License-Identifier: MIT */
/* The authors below grant copyright rights under the MIT license:
 * Copyright (c) 2025-2026 Qualcomm Technologies, Inc.
 */

// This implements XR_KHR_visibility_mask
// https://registry.khronos.org/OpenXR/specs/1.0/html/xrspec.html#XR_KHR_visibility_mask

// NOTE! openxr_view.cpp has part of the implementation for this extension!!!

#include "ext_management.h"
#include "../openxr_view.h"
#include "../../systems/defaults.h"

#define XR_EXT_FUNCTIONS( X )  \
	X(xrGetVisibilityMaskKHR)
OPENXR_DEFINE_FN_STATIC(XR_EXT_FUNCTIONS);

///////////////////////////////////////////

namespace sk {

///////////////////////////////////////////

typedef struct xr_visibility_mask_state_t {
	bool       available;
	material_t material;
} xr_visibility_mask_state_t;
static xr_visibility_mask_state_t local = { };

///////////////////////////////////////////

xr_system_ xr_ext_visibility_mask_init    (void*);
void       xr_ext_visibility_mask_shutdown(void*);
void       mask_poll                      (void*, XrEventDataBuffer* event);

void xr_ext_visibility_mask_update(XrViewConfigurationType view_type, uint32_t view_index);

///////////////////////////////////////////

void xr_ext_visibility_mask_register() {
	local = {};

	xr_system_t sys = {};
	sys.request_exts[sys.request_ext_count++] = XR_KHR_VISIBILITY_MASK_EXTENSION_NAME;
	sys.evt_initialize = { xr_ext_visibility_mask_init };
	sys.evt_shutdown   = { xr_ext_visibility_mask_shutdown };
	sys.evt_poll       = { (void (*)(void*, void*))mask_poll };
	ext_management_sys_register(sys);
}

///////////////////////////////////////////

xr_system_ xr_ext_visibility_mask_init(void*) {
	// Check if we got our extension
	if (!backend_openxr_ext_enabled(XR_KHR_VISIBILITY_MASK_EXTENSION_NAME))
		return xr_system_fail;

	// Load all extension functions
	OPENXR_LOAD_FN_RETURN(XR_EXT_FUNCTIONS, xr_system_fail);

	local.available = true;
	return xr_system_succeed;
}

///////////////////////////////////////////

void xr_ext_visibility_mask_shutdown(void*) {
	OPENXR_CLEAR_FN(XR_EXT_FUNCTIONS);

	material_release(local.material);
	local = {};
}

///////////////////////////////////////////

bool xr_ext_visibility_mask_available() {
	return local.available;
}

///////////////////////////////////////////

material_t xr_ext_visibility_mask_material() {
	if (local.material == nullptr) {
		// The mask mesh's winding order isn't guaranteed by the spec, and it's
		// drawn before the rest of the scene purely to write depth for an
		// early-z discard, so there's nothing to gain from culling it.
		local.material = material_create(sk_default_shader_visibility_mask);
		material_set_id          (local.material, "default/material_visibility_mask");
		material_set_depth_test  (local.material, depth_test_always);
		material_set_depth_write (local.material, true);
		material_set_cull        (local.material, cull_none);
		material_set_queue_offset(local.material, -1000);
	}
	return local.material;
}

///////////////////////////////////////////

void mask_poll(void*, XrEventDataBuffer* event) {
	if (event->type != XR_TYPE_EVENT_DATA_VISIBILITY_MASK_CHANGED_KHR)
		return;

	XrEventDataVisibilityMaskChangedKHR* mask_changed = (XrEventDataVisibilityMaskChangedKHR*)event;

	if (mask_changed->session != xr_session) {
		log_warnf("Visibility mask changed for a different session! (%p != %p)", mask_changed->session, xr_session);
		return;
	}

	XrViewConfigurationType view_type = mask_changed->viewConfigurationType;

	if (!xr_view_type_valid(view_type)) {
		log_warnf("Visibility mask event received for invalid view type: %d", view_type);
		return;
	}

	xr_ext_visibility_mask_update(view_type, mask_changed->viewIndex);
}

///////////////////////////////////////////

void xr_ext_visibility_mask_update(XrViewConfigurationType view_type, uint32_t view_index) {
	if (!local.available) return;

	XrVisibilityMaskKHR mask{XR_TYPE_VISIBILITY_MASK_KHR};
	if (XR_FAILED(xrGetVisibilityMaskKHR(xr_session, view_type, view_index, XR_VISIBILITY_MASK_TYPE_HIDDEN_TRIANGLE_MESH_KHR, &mask)))
		return;

	if (mask.vertexCountOutput != 0 && mask.indexCountOutput != 0) {
		vert_t*     verts = sk_malloc_t(vert_t,     mask.vertexCountOutput);
		vind_t*     inds  = sk_malloc_t(vind_t,     mask.indexCountOutput);
		XrVector2f* xr_v  = sk_malloc_t(XrVector2f, mask.vertexCountOutput);

		mask.vertexCapacityInput = mask.vertexCountOutput;
		mask.indexCapacityInput  = mask.indexCountOutput;
		mask.vertices = xr_v;
		mask.indices  = inds;

		if (XR_FAILED(xrGetVisibilityMaskKHR(xr_session, view_type, view_index, XR_VISIBILITY_MASK_TYPE_HIDDEN_TRIANGLE_MESH_KHR, &mask))) {
			sk_free(verts);
			sk_free(inds);
			sk_free(xr_v);
			return;
		}

		for (uint32_t i = 0; i < mask.vertexCountOutput; i++) {
			verts[i].pos  = {xr_v[i].x, xr_v[i].y, -1.0f};
			verts[i].norm = {0.0f, 0.0f, 1.0f};
			verts[i].col  = {(uint8_t)view_index, 0, 0, 255};
		}

		xr_set_visibility_mask(view_type, view_index, verts, mask.vertexCountOutput, inds, mask.indexCountOutput);

		sk_free(verts);
		sk_free(inds);
		sk_free(xr_v);
	}
}

} // namespace sk
