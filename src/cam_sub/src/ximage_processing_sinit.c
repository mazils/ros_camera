// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2020.2 (64-bit)
// Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef __linux__

#include "xstatus.h"
#include "xparameters.h"
#include "ximage_processing.h"

extern XImage_processing_Config XImage_processing_ConfigTable[];

XImage_processing_Config *XImage_processing_LookupConfig(u16 DeviceId) {
	XImage_processing_Config *ConfigPtr = NULL;

	int Index;

	for (Index = 0; Index < XPAR_XIMAGE_PROCESSING_NUM_INSTANCES; Index++) {
		if (XImage_processing_ConfigTable[Index].DeviceId == DeviceId) {
			ConfigPtr = &XImage_processing_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XImage_processing_Initialize(XImage_processing *InstancePtr, u16 DeviceId) {
	XImage_processing_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XImage_processing_LookupConfig(DeviceId);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XImage_processing_CfgInitialize(InstancePtr, ConfigPtr);
}

#endif

