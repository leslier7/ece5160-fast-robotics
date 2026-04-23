#ifndef MAPPING_H
#define MAPPING_H

#include "pid.h"

enum MappingStates{
    MAPPING_START,
    MAPPING_TURN,
    MAPPING_COLLECT,
    MAPPING_END
};

extern bool mapping_running;
extern int points_collected;

extern MappingStates MappingState;
extern PIDController mapping_pid;

void startMapping();

void mappingStateTick();

#endif