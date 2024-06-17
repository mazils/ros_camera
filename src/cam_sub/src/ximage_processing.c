// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
/***************************** Include Files *********************************/
#include "ximage_processing.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XImage_processing_CfgInitialize(XImage_processing *InstancePtr, XImage_processing_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Control_BaseAddress = ConfigPtr->Control_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

void XImage_processing_Start(XImage_processing *InstancePtr) {
    u32 Data;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XImage_processing_ReadReg(InstancePtr->Control_BaseAddress, XIMAGE_PROCESSING_CONTROL_ADDR_AP_CTRL) & 0x80;
    XImage_processing_WriteReg(InstancePtr->Control_BaseAddress, XIMAGE_PROCESSING_CONTROL_ADDR_AP_CTRL, Data | 0x01);
}

u32 XImage_processing_IsDone(XImage_processing *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XImage_processing_ReadReg(InstancePtr->Control_BaseAddress, XIMAGE_PROCESSING_CONTROL_ADDR_AP_CTRL);
    return (Data >> 1) & 0x1;
}

u32 XImage_processing_IsIdle(XImage_processing *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XImage_processing_ReadReg(InstancePtr->Control_BaseAddress, XIMAGE_PROCESSING_CONTROL_ADDR_AP_CTRL);
    return (Data >> 2) & 0x1;
}

u32 XImage_processing_IsReady(XImage_processing *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XImage_processing_ReadReg(InstancePtr->Control_BaseAddress, XIMAGE_PROCESSING_CONTROL_ADDR_AP_CTRL);
    // check ap_start to see if the pcore is ready for next input
    return !(Data & 0x1);
}

void XImage_processing_EnableAutoRestart(XImage_processing *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XImage_processing_WriteReg(InstancePtr->Control_BaseAddress, XIMAGE_PROCESSING_CONTROL_ADDR_AP_CTRL, 0x80);
}

void XImage_processing_DisableAutoRestart(XImage_processing *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XImage_processing_WriteReg(InstancePtr->Control_BaseAddress, XIMAGE_PROCESSING_CONTROL_ADDR_AP_CTRL, 0);
}

void XImage_processing_Set_in_r(XImage_processing *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XImage_processing_WriteReg(InstancePtr->Control_BaseAddress, XIMAGE_PROCESSING_CONTROL_ADDR_IN_R_DATA, (u32)(Data));
    XImage_processing_WriteReg(InstancePtr->Control_BaseAddress, XIMAGE_PROCESSING_CONTROL_ADDR_IN_R_DATA + 4, (u32)(Data >> 32));
}

u64 XImage_processing_Get_in_r(XImage_processing *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XImage_processing_ReadReg(InstancePtr->Control_BaseAddress, XIMAGE_PROCESSING_CONTROL_ADDR_IN_R_DATA);
    Data += (u64)XImage_processing_ReadReg(InstancePtr->Control_BaseAddress, XIMAGE_PROCESSING_CONTROL_ADDR_IN_R_DATA + 4) << 32;
    return Data;
}

void XImage_processing_Set_out_r(XImage_processing *InstancePtr, u64 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XImage_processing_WriteReg(InstancePtr->Control_BaseAddress, XIMAGE_PROCESSING_CONTROL_ADDR_OUT_R_DATA, (u32)(Data));
    XImage_processing_WriteReg(InstancePtr->Control_BaseAddress, XIMAGE_PROCESSING_CONTROL_ADDR_OUT_R_DATA + 4, (u32)(Data >> 32));
}

u64 XImage_processing_Get_out_r(XImage_processing *InstancePtr) {
    u64 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XImage_processing_ReadReg(InstancePtr->Control_BaseAddress, XIMAGE_PROCESSING_CONTROL_ADDR_OUT_R_DATA);
    Data += (u64)XImage_processing_ReadReg(InstancePtr->Control_BaseAddress, XIMAGE_PROCESSING_CONTROL_ADDR_OUT_R_DATA + 4) << 32;
    return Data;
}

void XImage_processing_InterruptGlobalEnable(XImage_processing *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XImage_processing_WriteReg(InstancePtr->Control_BaseAddress, XIMAGE_PROCESSING_CONTROL_ADDR_GIE, 1);
}

void XImage_processing_InterruptGlobalDisable(XImage_processing *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XImage_processing_WriteReg(InstancePtr->Control_BaseAddress, XIMAGE_PROCESSING_CONTROL_ADDR_GIE, 0);
}

void XImage_processing_InterruptEnable(XImage_processing *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XImage_processing_ReadReg(InstancePtr->Control_BaseAddress, XIMAGE_PROCESSING_CONTROL_ADDR_IER);
    XImage_processing_WriteReg(InstancePtr->Control_BaseAddress, XIMAGE_PROCESSING_CONTROL_ADDR_IER, Register | Mask);
}

void XImage_processing_InterruptDisable(XImage_processing *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XImage_processing_ReadReg(InstancePtr->Control_BaseAddress, XIMAGE_PROCESSING_CONTROL_ADDR_IER);
    XImage_processing_WriteReg(InstancePtr->Control_BaseAddress, XIMAGE_PROCESSING_CONTROL_ADDR_IER, Register & (~Mask));
}

void XImage_processing_InterruptClear(XImage_processing *InstancePtr, u32 Mask) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XImage_processing_WriteReg(InstancePtr->Control_BaseAddress, XIMAGE_PROCESSING_CONTROL_ADDR_ISR, Mask);
}

u32 XImage_processing_InterruptGetEnabled(XImage_processing *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XImage_processing_ReadReg(InstancePtr->Control_BaseAddress, XIMAGE_PROCESSING_CONTROL_ADDR_IER);
}

u32 XImage_processing_InterruptGetStatus(XImage_processing *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XImage_processing_ReadReg(InstancePtr->Control_BaseAddress, XIMAGE_PROCESSING_CONTROL_ADDR_ISR);
}

