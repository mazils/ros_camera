// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef XIMAGE_PROCESSING_H
#define XIMAGE_PROCESSING_H

#ifdef __cplusplus
extern "C" {
#endif

/***************************** Include Files *********************************/
#ifndef __linux__
#include "xil_types.h"
#include "xil_assert.h"
#include "xstatus.h"
#include "xil_io.h"
#else
#include <stdint.h>
#include <assert.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stddef.h>
#endif
#include "ximage_processing_hw.h"

/**************************** Type Definitions ******************************/
#ifdef __linux__
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#else
typedef struct {
    u16 DeviceId;
    u32 Control_BaseAddress;
} XImage_processing_Config;
#endif

typedef struct {
    u64 Control_BaseAddress;
    u32 IsReady;
} XImage_processing;

typedef u32 word_type;

/***************** Macros (Inline Functions) Definitions *********************/
#ifndef __linux__
#define XImage_processing_WriteReg(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define XImage_processing_ReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))
#else
#define XImage_processing_WriteReg(BaseAddress, RegOffset, Data) \
    *(volatile u32*)((BaseAddress) + (RegOffset)) = (u32)(Data)
#define XImage_processing_ReadReg(BaseAddress, RegOffset) \
    *(volatile u32*)((BaseAddress) + (RegOffset))

#define Xil_AssertVoid(expr)    assert(expr)
#define Xil_AssertNonvoid(expr) assert(expr)

#define XST_SUCCESS             0
#define XST_DEVICE_NOT_FOUND    2
#define XST_OPEN_DEVICE_FAILED  3
#define XIL_COMPONENT_IS_READY  1
#endif

/************************** Function Prototypes *****************************/
#ifndef __linux__
int XImage_processing_Initialize(XImage_processing *InstancePtr, u16 DeviceId);
XImage_processing_Config* XImage_processing_LookupConfig(u16 DeviceId);
int XImage_processing_CfgInitialize(XImage_processing *InstancePtr, XImage_processing_Config *ConfigPtr);
#else
int XImage_processing_Initialize(XImage_processing *InstancePtr, const char* InstanceName);
int XImage_processing_Release(XImage_processing *InstancePtr);
#endif

void XImage_processing_Start(XImage_processing *InstancePtr);
u32 XImage_processing_IsDone(XImage_processing *InstancePtr);
u32 XImage_processing_IsIdle(XImage_processing *InstancePtr);
u32 XImage_processing_IsReady(XImage_processing *InstancePtr);
void XImage_processing_EnableAutoRestart(XImage_processing *InstancePtr);
void XImage_processing_DisableAutoRestart(XImage_processing *InstancePtr);

void XImage_processing_Set_in_r(XImage_processing *InstancePtr, u64 Data);
u64 XImage_processing_Get_in_r(XImage_processing *InstancePtr);
void XImage_processing_Set_out_r(XImage_processing *InstancePtr, u64 Data);
u64 XImage_processing_Get_out_r(XImage_processing *InstancePtr);

void XImage_processing_InterruptGlobalEnable(XImage_processing *InstancePtr);
void XImage_processing_InterruptGlobalDisable(XImage_processing *InstancePtr);
void XImage_processing_InterruptEnable(XImage_processing *InstancePtr, u32 Mask);
void XImage_processing_InterruptDisable(XImage_processing *InstancePtr, u32 Mask);
void XImage_processing_InterruptClear(XImage_processing *InstancePtr, u32 Mask);
u32 XImage_processing_InterruptGetEnabled(XImage_processing *InstancePtr);
u32 XImage_processing_InterruptGetStatus(XImage_processing *InstancePtr);

#ifdef __cplusplus
}
#endif

#endif
