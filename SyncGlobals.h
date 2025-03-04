#ifndef SyncGlobals_H
#define SyncGlobals_H

#include "pico/mutex.h"
#include "pico/critical_section.h"

static mutex_t g_eepromMutex;
static critical_section_t g_eepromCritSection;
static critical_section_t g_eepromCritSection2;

static int g_dmaChanToStopA = 0;
static int g_dmaChanToStop32 = 0;
#endif