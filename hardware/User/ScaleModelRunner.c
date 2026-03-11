#include "ScaleModelRunner.h"

#include <math.h>

// ========== 模型输出快照（全局） ==========
volatile float ScaleModel_yFused = 0.0f;
volatile float ScaleModel_yBaseline = 0.0f;
volatile float ScaleModel_yModel = NAN;
volatile u8 ScaleModel_ready = 0;

#ifdef ENABLE_SCALE_MODEL

// 注意：此头文件在model目录下；这里用相对路径，避免依赖工程解释路径配置
#include "../model/scale_model_fp32.h"

#include <string.h>

// ========== 模型流式状态（静态分配，避免动态内存） ==========
static scale_stream_t gScaleStream;
static u8 gScaleStreamInited = 0;

void ScaleModelRunner_Init(void)
{
    scale_stream_init(&gScaleStream);
    gScaleStreamInited = 1;
}

u8 ScaleModelRunner_Step(u32 pressureRaw,
                         const double acc[3],
                         const double gyro[3],
                         const double quat[4],
                         float* yFusedOut,
                         float* yBaselineOut,
                         float* yModelOut)
{
    if (gScaleStreamInited == 0)
    {
        ScaleModelRunner_Init();
    }

    // 组装一帧原始特征（必须与训练CSV的单位/顺序一致）
    float xRaw[SCALE_MODEL_CHANNELS];
    xRaw[0]  = (float)pressureRaw;
    xRaw[1]  = (float)acc[0];
    xRaw[2]  = (float)acc[1];
    xRaw[3]  = (float)acc[2];
    xRaw[4]  = (float)gyro[0];
    xRaw[5]  = (float)gyro[1];
    xRaw[6]  = (float)gyro[2];
    xRaw[7]  = (float)quat[0];  // q_w
    xRaw[8]  = (float)quat[1];  // q_x
    xRaw[9]  = (float)quat[2];  // q_y
    xRaw[10] = (float)quat[3];  // q_z

    scale_step_out_t out;
    scale_stream_step(&gScaleStream, xRaw, &out);

    // 更新全局快照，供USB串口输出使用
    ScaleModel_yFused = out.y_fused;
    ScaleModel_yBaseline = out.y_baseline;
    ScaleModel_yModel = out.y_model;
    ScaleModel_ready = out.model_ready ? 1 : 0;

    if (yFusedOut != NULL)
    {
        *yFusedOut = out.y_fused;
    }
    if (yBaselineOut != NULL)
    {
        *yBaselineOut = out.y_baseline;
    }
    if (yModelOut != NULL)
    {
        *yModelOut = out.y_model;
    }

    return out.model_ready ? 1 : 0;
}

#else

// 未启用模型：保留空实现，保证不影响现有业务与链接
void ScaleModelRunner_Init(void)
{
}

u8 ScaleModelRunner_Step(u32 pressureRaw,
                         const double acc[3],
                         const double gyro[3],
                         const double quat[4],
                         float* yFusedOut,
                         float* yBaselineOut,
                         float* yModelOut)
{
    (void)pressureRaw;
    (void)acc;
    (void)gyro;
    (void)quat;

    if (yFusedOut != NULL)
    {
        *yFusedOut = 0.0f;
    }
    if (yBaselineOut != NULL)
    {
        *yBaselineOut = 0.0f;
    }
    if (yModelOut != NULL)
    {
        *yModelOut = NAN;
    }

    // 未启用模型时，同步全局快照
    ScaleModel_yFused = 0.0f;
    ScaleModel_yBaseline = 0.0f;
    ScaleModel_yModel = NAN;
    ScaleModel_ready = 0;

    return 0;
}

#endif


