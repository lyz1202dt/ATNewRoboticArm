#ifndef __DATA_PACK_H__
#define __DATA_PACK_H__

#include <stdint.h>

#pragma pack(push, 1)

typedef struct{
    float position;
    float velocity;
    float torque;
}MotorCmdPack;

typedef struct{
    float position;
    float velocity;
    float torque;
    float kp;
    float kd;
    float ki;
}MotorCmdPackEx;

typedef struct{
    float position;
    float velocity;
    float torque;
}MotorStatePack;

typedef struct{
    uint8_t head;   //0x5A
    MotorCmdPack motor[6];
}MCUTarget1Pack;

typedef struct{
    uint8_t head;   //0x5B
    MotorCmdPackEx motor[6];
}MCUTarget2Pack;

typedef struct{
    uint8_t head;   //0x4A
    MotorStatePack motor[6];
}MCUStatePack;

#pragma pack(pop)
#endif