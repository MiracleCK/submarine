#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

#define THISFIRMWARE "F1 v1.0.2dev"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 1,0,2,FIRMWARE_VERSION_TYPE_DEV

#define FW_MAJOR 1
#define FW_MINOR 0
#define FW_PATCH 2
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV
