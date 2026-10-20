#pragma once
struct AccelStepper { enum {DRIVER=1}; AccelStepper(int,int,int); void setSpeed(float); bool runSpeed(); void setMaxSpeed(float); void setMinPulseWidth(unsigned int); };
