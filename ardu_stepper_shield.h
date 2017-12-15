#ifndef __ARDU_STEPPER_SHIELD_H
#define __ARDU_STEPPER_SHIELD_H

#include <stdint.h>
#include <stdbool.h>

#define MCS_PIN_DIR_LATCH   24
#define MCS_PIN_PWM2A       23
#define MCS_PIN_PWM1B       22
#define MCS_PIN_PWM1A       20
#define MCS_PIN_DIR_SER     19
#define MCS_PIN_DIR_EN      18
#define MCS_PIN_PWM0A       17
#define MCS_PIN_PWM0B       16
#define MCS_PIN_DIR_CLK     15
#define MCS_PIN_PWM2B       14

#define MODE_IDLE 0
#define MODE_FWD 1
#define MODE_IDLEH 2
#define MODE_REV 3

#define M1_FWD 0x10
#define M1_REV 0x20
#define M2_FWD 0x08
#define M2_REV 0x40

#define M3_FWD 0x01
#define M3_REV 0x04
#define M4_FWD 0x02
#define M4_REV 0x80

typedef enum {DIR_NONE, DIR_FORWARD, DIR_BACKWARD, DIR_TURN_RIGHT, DIR_TURN_LEFT, DIR_INVALID} step_direction_e;

#define DIRECTION_VALID(a) ((a) != DIR_NONE && (a) < DIR_INVALID)

typedef struct
{


}stepper_config_t;

void stepper_init(stepper_config_t *config);

void step_a(bool forward);

void step_b(bool forward);

void step_ab(step_direction_e direction);

void step_release_all(void);

void step_cont_start(uint32_t interval_ms, bool forward);

void step_process_command(const uint8_t *ptr, uint32_t length);

#endif
