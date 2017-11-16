#ifndef __STEPPER_CONTROLLER_H
#define __STEPPER_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"

typedef struct
{
    uint32_t pin_ch_a;
    uint32_t pin_ch_an;
    uint32_t pin_ch_b;
    uint32_t pin_ch_bn;
    uint32_t pwm_ctrl_index;
}stpctrl_stepper_config_t;

typedef struct
{
    stpctrl_stepper_config_t stepper_1;

}stpctrl_config_t;

void stepper_controller_init(stpctrl_config_t *config);

void stepper_controller_stepper_set(uint32_t stepper_index, uint32_t stepper_position);

#endif
