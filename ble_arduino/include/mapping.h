#ifndef MAPPING_H
#define MAPPING_H

#include "pid.h"

extern bool mapping_running;


enum MappingStates{
    START,
    TURN,
    COLLECT,
    END
};

void startMapping();

void mappingStateTick();

#endif