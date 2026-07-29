#pragma once

#include "../libraries/miniaudio.h"
#include "../libraries/ferr_thread.h"
#include "assets.h"

namespace sk {

typedef enum sound_data_ {
	sound_data_none = 0,
	sound_data_pcm,         // Fully decoded/generated samples in memory
	sound_data_stream_file, // Compressed source bytes, decoded per-voice
	sound_data_ring,        // Live ring buffer, mic or user fed
} sound_data_;

struct _sound_t {
	asset_header_t  header;
	sound_data_     data_type;
	sound_channels_ channels;   // Interleaved when more than one
	float           decibels;
	float           norm_gain;  // Loudness normalization from load-time RMS
	int32_t         sample_rate;
	bool32_t        fuma;       // Stream bytes are FuMa order, convert on read

	// sound_data_pcm
	float*         pcm;
	uint64_t       pcm_count;   // Frames

	// sound_data_stream_file, bytes owned by the sound
	void*          file_data;
	size_t         file_size;
	uint64_t       file_frames; // Decoded length

	// sound_data_ring
	ma_pcm_rb      ring;
	float*         ring_data;
	uint64_t       ring_capacity;
	uint64_t       ring_written; // Total written, capped at capacity
	ft_mutex_t     ring_lock;
};

void  sound_destroy      (sound_t sound);
float decibels_to_signal (float decibel);
void  sound_fuma_to_ambix(float* frames, uint64_t frame_count);

inline int32_t sound_channel_count(sound_channels_ channels) {
	switch (channels) {
	case sound_channels_stereo:     return 2;
	case sound_channels_ambisonic1: return 4;
	default:                        return 1;
	}
}

} // namespace sk
