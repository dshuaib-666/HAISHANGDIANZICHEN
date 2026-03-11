#ifndef SCALE_MODEL_FP32_H
#define SCALE_MODEL_FP32_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Feature order (raw input must match training):
// pressure, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, q_w, q_x, q_y, q_z

#define SCALE_MODEL_WINDOW_SIZE (100)
#define SCALE_MODEL_CHANNELS (11)
#define SCALE_BASELINE_WINDOW_SIZE (100)

// model_type:
//   0 = direct_mlp
//   1 = baseline_factor
//   2 = baseline_residual
#define SCALE_MODEL_TYPE (1)

typedef struct {
  float gyro_on;
  float gyro_off;
  float acc_dev_on;
  float acc_dev_off;
  float g;
  // tilt compensation (for long, large tilt in calm conditions)
  // Use cos(theta) thresholds to avoid expensive acosf:
  // - enable when cos < tilt_cos_on  (tilt angle larger)
  // - disable when cos > tilt_cos_off (tilt angle smaller)
  // Recommended defaults in scale_stream_init():
  //   tilt_cos_on  = 0.990f  (~8 deg)
  //   tilt_cos_off = 0.995f  (~5.7 deg)
  //   tilt_cos_min = 0.700f  (cap at ~45 deg to avoid blow-up)
  float tilt_cos_on;
  float tilt_cos_off;
  float tilt_cos_min;
} scale_fusion_cfg_t;

typedef struct {
  // baseline (pressure sliding mean) buffer
  float p_buf[SCALE_BASELINE_WINDOW_SIZE];
  float p_sum;
  uint16_t p_pos;
  uint16_t p_filled;

  // normalized feature ring buffer for model window
  float x_norm[SCALE_MODEL_WINDOW_SIZE][SCALE_MODEL_CHANNELS];
  uint16_t x_pos;
  uint16_t x_filled;

  // fused hysteresis state
  uint8_t use_model_gate;
  // tilt compensation hysteresis state
  uint8_t use_tilt_comp;
  scale_fusion_cfg_t fuse;
} scale_stream_t;

typedef struct {
  float y_baseline;  // baseline output (pressure sliding mean + linear fit)
  float y_model;     // model output (NaN if not ready)
  float y_fused;     // fused output (baseline vs model)
  uint8_t model_ready;
  uint8_t use_model_gate;
} scale_step_out_t;

// Initialize stream state (call once).
void scale_stream_init(scale_stream_t* st);

// Process one new raw sample (one frame, length = SCALE_MODEL_CHANNELS).
// x_raw must be in FEATURE_COLS order and in the SAME units as training CSV.
void scale_stream_step(scale_stream_t* st, const float x_raw[SCALE_MODEL_CHANNELS], scale_step_out_t* out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // SCALE_MODEL_FP32_H
