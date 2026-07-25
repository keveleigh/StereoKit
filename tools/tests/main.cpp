// SPDX-License-Identifier: MIT
// The authors below grant copyright rights under the MIT license:
// Copyright (c) 2026 Nick Klingensmith

// Headless native test runner for StereoKit's C API. Each suite
// self-initializes StereoKit in offscreen mode and returns its failure count,
// so this doubles as a CI gate. New native suites get one line in main.
//
// No arguments runs every test suite; a flag opts just that suite in (and
// flags combine). The benchmark is a measurement, not a test, so it's opt-in.

#include "audio_tests.h"

#include <stdio.h>
#include <string.h>

static bool has_flag(int argc, char** argv, const char* flag) {
	for (int i = 1; i < argc; i++)
		if (strcmp(argv[i], flag) == 0)
			return true;
	return false;
}

int main(int argc, char* argv[]) {
	bool all      = argc <= 1;
	int  failures = 0;

	if (all || has_flag(argc, argv, "-audiotest"))   failures += audio_tests_run();
	if (all || has_flag(argc, argv, "-audiostress")) failures += audio_stress_run();

	if (has_flag(argc, argv, "-audiobench")) failures += audio_bench_run();

	printf("=== SKTests: %d total failure(s) ===\n", failures);
	return failures;
}
