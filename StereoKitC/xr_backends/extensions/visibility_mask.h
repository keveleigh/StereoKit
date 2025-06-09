/* SPDX-License-Identifier: MIT */
/* The authors below grant copyright rights under the MIT license:
 * Copyright (c) 2025-2026 Qualcomm Technologies, Inc.
 */

#pragma once

namespace sk {

void       xr_ext_visibility_mask_register ();
bool       xr_ext_visibility_mask_available();
material_t xr_ext_visibility_mask_material ();

void xr_ext_visibility_mask_update(XrViewConfigurationType view_type, uint32_t view_index);
}