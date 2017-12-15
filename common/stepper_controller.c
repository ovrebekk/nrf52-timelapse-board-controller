#include "stepper_controller.h"

#define STP_PWM NRF_PWM0

static uint16_t stepper_motor_sequence[] = {255, 0,   0,   255,
                                           255, 0,   0,   0,
                                           255, 255, 0,   0,
                                           0,   255, 0,   0,
                                           0,   255, 255, 0,
                                           0,   0,   255, 0,
                                           0,   0,   255, 255, 
                                           0,   0,   0,   255};


void stepper_controller_init(stpctrl_config_t *config)
{
    STP_PWM->ENABLE = PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos;
    STP_PWM->COUNTERTOP = 255 << PWM_COUNTERTOP_COUNTERTOP_Pos;
    STP_PWM->MODE = PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos;
    STP_PWM->DECODER = PWM_DECODER_MODE_NextStep << PWM_DECODER_MODE_Pos | PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos;
    STP_PWM->PRESCALER = PWM_PRESCALER_PRESCALER_DIV_16 << PWM_PRESCALER_PRESCALER_Pos;
    //STP_PWM->SHORTS = PWM_SHORTS_LOOPSDONE_SEQSTART0_Msk | PWM_SHORTS_SEQEND0_STOP_Msk;
    STP_PWM->LOOP = 0;
    STP_PWM->PSEL.OUT[0] = config->stepper_1.pin_ch_a;
    STP_PWM->PSEL.OUT[1] = config->stepper_1.pin_ch_an;
    STP_PWM->PSEL.OUT[2] = config->stepper_1.pin_ch_b;
    STP_PWM->PSEL.OUT[3] = config->stepper_1.pin_ch_bn;
    STP_PWM->SEQ[0].PTR = (uint32_t)stepper_motor_sequence;
    STP_PWM->SEQ[0].CNT = sizeof(stepper_motor_sequence) / sizeof(stepper_motor_sequence[0]);
    STP_PWM->SEQ[0].REFRESH = 0;
    STP_PWM->SEQ[0].ENDDELAY = 0;
    STP_PWM->TASKS_SEQSTART[0] = 1;

    NRF_PPI->CH[0].EEP = (uint32_t)&STP_PWM->EVENTS_SEQEND[0];
    NRF_PPI->CH[0].TEP = (uint32_t)&STP_PWM->TASKS_SEQSTART[0];
    NRF_PPI->CHENSET = 1 << 0;

}

void stepper_controller_stepper_set(uint32_t stepper_index, uint32_t stepper_position)
{
    STP_PWM->TASKS_NEXTSTEP = 1;
}

