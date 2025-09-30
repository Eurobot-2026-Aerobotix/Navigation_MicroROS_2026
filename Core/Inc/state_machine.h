#ifndef __STATE_MACHINE_H__
#define __STATE_MACHINE_H__

#include <stdint.h>

// State machine definitions
typedef enum {
    IDLE = 0,
    MOVING_FORWARD,
    MOVING_BACKWARD,
    ROTATIN,
    CURVE,
    GOTO_POSITION,
    EMERGENCY_STOP,
    COMPLETED,
} robot_state_t;

typedef struct {
    robot_state_t current_state;
    float param1, param2, param3, param4, param5, param6, param7, param8, param9; // 9 parameters for operations
} state_machine;

// Global state machine instance
extern state_machine robot_state_machine;

// State machine functions
void state_machine_init(void);
void state_machine_set_state(robot_state_t new_state, float p1, float p2, float p3, float p4, float p5, float p6, float p7, float p8, float p9);
uint8_t state_machine_is_busy(void);
uint8_t state_machine_is_complete(void);
void state_machine_task(void);

#endif /* __STATE_MACHINE_H__ */
