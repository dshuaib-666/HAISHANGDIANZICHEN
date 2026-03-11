#ifndef __SCALE_MODEL_RUNNER_H
#define __SCALE_MODEL_RUNNER_H

#include "Config.h"

// ========== 模型输出快照（用于USB串口输出/调试） ==========
// 说明：这些变量会在 ScaleModelRunner_Step() 内部更新
extern volatile float ScaleModel_yFused;
extern volatile float ScaleModel_yBaseline;
extern volatile float ScaleModel_yModel;
extern volatile u8 ScaleModel_ready;

/**
 * @brief 初始化深度学习模型流式推理状态
 * @note 仅在启用ENABLE_SCALE_MODEL时生效；未启用时函数为空实现
 */
void ScaleModelRunner_Init(void);

/**
 * @brief 输入一帧数据，执行一次流式推理
 * @param pressureRaw  压力原始值（建议使用ADC原始计数，单位/量纲必须与训练数据一致）
 * @param acc          三轴加速度（单位/量纲必须与训练数据一致）
 * @param gyro         三轴角速度（单位/量纲必须与训练数据一致）
 * @param quat         四元素（q_w,q_x,q_y,q_z，单位/量纲必须与训练数据一致）
 * @param yFusedOut    融合输出（可为NULL）
 * @param yBaselineOut 基线输出（可为NULL）
 * @param yModelOut    模型输出（可为NULL，未就绪时会返回NaN）
 * @return 1=模型窗口已满，输出有效；0=尚未就绪
 *
 * @note 模型特征顺序固定为：
 *       pressure, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, q_w, q_x, q_y, q_z
 */
u8 ScaleModelRunner_Step(u32 pressureRaw,
                         const double acc[3],
                         const double gyro[3],
                         const double quat[4],
                         float* yFusedOut,
                         float* yBaselineOut,
                         float* yModelOut);

#endif


