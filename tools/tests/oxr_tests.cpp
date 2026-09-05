// SPDX-License-Identifier: MIT
// The authors below grant copyright rights under the MIT license:
// Copyright (c) 2026 Nick Klingensmith

#include "oxr_tests.h"

#include <stereokit.h>
#include <xr_backends/extensions/ext_management.h>

#include <stdio.h>
#include <string.h>

using namespace sk;

static int oxr_failures = 0;

#define OXR_CHECK(condition, description) do { \
	if (condition) { log_infof("[oxr_test] pass: %s", description); } \
	else           { log_errf ("[oxr_test] FAIL: %s", description); oxr_failures += 1; } \
	} while (0)

struct test_cb_data_t {
	int32_t call_count;
	void*   received_context;
	void*   received_session_info;
	bool    return_value;
};

static bool32_t oxr_test_callback_single(void* context, void* session_create_info) {
	test_cb_data_t* data = (test_cb_data_t*)context;
	if (data) {
		data->call_count++;
		data->received_context      = context;
		data->received_session_info = session_create_info;
		return data->return_value ? 1 : 0;
	}
	return 1;
}

static char oxr_order_buf[16];
static bool32_t oxr_test_callback_order_a(void*, void*) {
	strcat(oxr_order_buf, "A");
	return 1;
}
static bool32_t oxr_test_callback_order_b(void*, void*) {
	strcat(oxr_order_buf, "B");
	return 1;
}
static bool32_t oxr_test_callback_order_c(void*, void*) {
	strcat(oxr_order_buf, "C");
	return 1;
}
static bool32_t oxr_test_callback_order_fail(void*, void*) {
	strcat(oxr_order_buf, "X");
	return 0;
}

int oxr_tests_run() {
	oxr_failures = 0;

	// --- Test 1: Callback invocation, context passing, and session_info passing ---
	{
		test_cb_data_t data = { 0, nullptr, nullptr, true };
		XrSessionCreateInfo dummy_info = {};
		dummy_info.type = XR_TYPE_SESSION_CREATE_INFO;

		backend_openxr_add_callback_pre_session_create(oxr_test_callback_single, &data);

		bool result = ext_management_evt_pre_session_create(&dummy_info);
		OXR_CHECK(result == true, "evt_pre_session_create succeeded");
		OXR_CHECK(data.call_count == 1, "callback invoked exactly once");
		OXR_CHECK(data.received_context == &data, "callback received correct context pointer");
		OXR_CHECK(data.received_session_info == (void*)&dummy_info, "callback received correct XrSessionCreateInfo pointer");

		// Subsequent call should have no callbacks since the array was freed
		data = { 0, nullptr, nullptr, true };
		result = ext_management_evt_pre_session_create(&dummy_info);
		OXR_CHECK(result == true, "subsequent evt_pre_session_create succeeded");
		OXR_CHECK(data.call_count == 0, "callbacks array was consumed and not called again");
	}

	// --- Test 2: Returning false from a callback aborts session creation ---
	{
		test_cb_data_t data = { 0, nullptr, nullptr, false };
		XrSessionCreateInfo dummy_info = {};
		dummy_info.type = XR_TYPE_SESSION_CREATE_INFO;

		backend_openxr_add_callback_pre_session_create(oxr_test_callback_single, &data);

		bool result = ext_management_evt_pre_session_create(&dummy_info);
		OXR_CHECK(result == false, "evt_pre_session_create returned false when callback returned false");
		OXR_CHECK(data.call_count == 1, "failing callback was invoked");
	}

	// --- Test 3: Multiple callbacks run in order, and early failure stops chain ---
	{
		oxr_order_buf[0] = '\0';
		XrSessionCreateInfo dummy_info = {};
		dummy_info.type = XR_TYPE_SESSION_CREATE_INFO;

		backend_openxr_add_callback_pre_session_create(oxr_test_callback_order_a, nullptr);
		backend_openxr_add_callback_pre_session_create(oxr_test_callback_order_b, nullptr);
		backend_openxr_add_callback_pre_session_create(oxr_test_callback_order_c, nullptr);

		bool result = ext_management_evt_pre_session_create(&dummy_info);
		OXR_CHECK(result == true, "multiple callbacks succeeded");
		OXR_CHECK(strcmp(oxr_order_buf, "ABC") == 0, "callbacks executed in order of registration");

		// Test early failure stopping chain
		oxr_order_buf[0] = '\0';
		backend_openxr_add_callback_pre_session_create(oxr_test_callback_order_a, nullptr);
		backend_openxr_add_callback_pre_session_create(oxr_test_callback_order_fail, nullptr);
		backend_openxr_add_callback_pre_session_create(oxr_test_callback_order_c, nullptr);

		result = ext_management_evt_pre_session_create(&dummy_info);
		OXR_CHECK(result == false, "chain with failing callback returned false");
		OXR_CHECK(strcmp(oxr_order_buf, "AX") == 0, "callback after failing callback was not executed");
	}

	// --- Test 4: Post-init guard ---
	{
		sk_settings_t settings = {};
		settings.app_name = "SKTests OXR";
		settings.mode     = app_mode_offscreen;
		if (sk_init(settings)) {
			// Calling backend_openxr_add_callback_pre_session_create after sk_init must safely reject without crashing
			backend_openxr_add_callback_pre_session_create(oxr_test_callback_single, nullptr);
			sk_shutdown();
			OXR_CHECK(true, "StereoKit shutdown successfully");
		}
	}

	// --- Test 5: Lifecycle cleanup when session is not created ---
	{
		test_cb_data_t data = { 0, nullptr, nullptr, true };
		backend_openxr_add_callback_pre_session_create(oxr_test_callback_single, &data);

		sk_settings_t settings = {};
		settings.app_name = "SKTests OXR Cleanup";
		settings.mode     = app_mode_offscreen;
		if (sk_init(settings)) {
			sk_shutdown();
			OXR_CHECK(data.call_count == 0, "callbacks were safely freed without invocation during offscreen shutdown");
		}
	}

	return oxr_failures;
}
