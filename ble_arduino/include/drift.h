#ifndef DRIFT_H
#define DRIFT_H

extern bool drift_running;

enum DriftStates{
    START,
    TOWARD_WALL,
    DRIFT,
    RETURN,
    END
};

void startDrift();

void driftStateTick();

#endif