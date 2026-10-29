/*
 * sfra_task.h
 *
 * SFRA 扫频任务的对外接口（实现见 sfra_task.c）。
 * 只在 SFRA_ENABLE == 1 时有实际内容，否则两个函数都是空。
 */

#ifndef SFRA_TASK_H
#define SFRA_TASK_H

#include "DSP28x_Project.h"
#include "dcac_cfg.h"

// 初始化 SFRA 库对象、结果数组、映射表和 SCI-A。
// 必须在中断使能之前调用。
void Sfra_Task_Init(void);

// 主循环里每圈调用一次：跑库的后台状态机、处理自动触发和扫完判定。
void Sfra_Task_Background(void);

#endif
