#pragma once

// Deterministic offline audio tests, run with `SKTests -audiotest`.
// Returns 0 on success, the number of failed checks otherwise.
int audio_tests_run();

// Live-device concurrency stress, run with `SKTests -audiostress`.
// Meant for TSan/ASan builds, hammers the voice API against the real audio
// callback for ~10s. Success is completing without a crash or sanitizer
// report.
int audio_stress_run();

// Offline mixer performance benchmark, run with `SKTests -audiobench`.
// Times the audio thread's full block render across voice-load scenarios,
// reporting cost as a percentage of real-time. Build Release for real numbers.
int audio_bench_run();
