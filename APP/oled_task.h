/*
 * oled_task.h
 *
 * OLED 显示任务（从 main.c 搬出来的，见 oled_task.c 的文件头说明）。
 *
 * 只在 SFRA_ENABLE == 0（OLED 版）时有实际内容；扫频版里两个函数都是空。
 */

#ifndef OLED_TASK_H
#define OLED_TASK_H

#include "DSP28x_Project.h"
#include "dcac_cfg.h"

// 打开并清屏。要在 ePWM / 中断启动之前调用。
void Oled_Task_Init(void);

// 后台循环里调用。内部按 isrcnt 自己做 500ms 节流，所以可以每圈都调。
//   isrcnt : 当前 ADC 中断计数（main.c 的 IsrCnt）
//   fault  : 当前故障码（FAULT_NONE / FAULT_UV / FAULT_OC，见 protect.h）
void Oled_Task_Run(Uint16 isrcnt, Uint16 fault);

#endif
