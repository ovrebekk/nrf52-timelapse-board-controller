#include "ardu_stepper_shield.h"
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_timer.h"
#include "nrf_drv_pwm.h"

static uint8_t pinmask_m1[4] = {0x00, 0x04, 0x08, 0x0C};
static uint8_t pinmask_m2[4] = {0x00, 0x02, 0x10, 0x12};
static uint8_t pinmask_m3[4] = {0x00, 0x01, 0x40, 0x41};
static uint8_t pinmask_m4[4] = {0x00, 0x20, 0x80, 0xA0};

static uint8_t sequence[] = {M3_FWD | M4_FWD, M3_REV | M4_FWD, M3_REV | M4_REV, M3_FWD | M4_REV};
static uint8_t sequence1[] = {M1_FWD | M2_FWD, M1_REV | M2_FWD, M1_REV | M2_REV, M1_FWD | M2_REV};

static uint32_t m_pos_stepper_a = 0, m_pos_stepper_b = 0;

static const nrf_drv_spi_t m_spi_master_0 = NRF_DRV_SPI_INSTANCE(0);

static uint32_t ret_code;

static int m_run_direction;
static uint32_t m_stepper_on_period = 50, m_stepper_cont_interval;
static uint8_t m_stepper_duty_cycle = 255;

// Method B
static bool m_left_forward, m_right_forward;
static int32_t m_left_interval_us, m_right_interval_us;

static nrf_pwm_values_individual_t m_pwm_stepper_seq_values;
static nrf_pwm_sequence_t const    m_pwm_stepper_seq =
{
    .values.p_individual = &m_pwm_stepper_seq_values,
    .length              = NRF_PWM_VALUES_LENGTH(m_pwm_stepper_seq_values),
    .repeats             = 0,
    .end_delay           = 0
};
static nrf_drv_pwm_t m_pwm_stepper_control = NRF_DRV_PWM_INSTANCE(0);

APP_TIMER_DEF(m_app_timer_step_delayed_release);
APP_TIMER_DEF(m_app_timer_step_left);
APP_TIMER_DEF(m_app_timer_step_right);

void pwm_stepper_duty_cycle_set(uint8_t duty);

static void write_byte(uint8_t byte)
{
    nrf_gpio_pin_clear(MCS_PIN_DIR_LATCH);
    nrf_delay_us(5);
    for(int i = 0; i < 8; i++)
    {
        (byte & 0x01) ? nrf_gpio_pin_set(MCS_PIN_DIR_SER) : nrf_gpio_pin_clear(MCS_PIN_DIR_SER);
        nrf_delay_us(5);
        nrf_gpio_pin_set(MCS_PIN_DIR_CLK);
        nrf_delay_us(5);
        nrf_gpio_pin_clear(MCS_PIN_DIR_CLK);
        nrf_delay_us(5);
        byte >>= 1;
    }
    nrf_delay_us(5);
    nrf_gpio_pin_set(MCS_PIN_DIR_LATCH);
}

static void write_byte_spi(uint8_t byte)
{
    uint8_t step_spi_buf[2];
    step_spi_buf[0] = step_spi_buf[1] = byte;
    ret_code = nrf_drv_spi_transfer(&m_spi_master_0, step_spi_buf, 2, 0, 0);
}

static void pwm_event_handler(nrf_drv_pwm_evt_type_t event_type)
{
    if (event_type == NRF_DRV_PWM_EVT_FINISHED)
    {
 
    }
}

static void enable_pwms(bool enable)
{
    nrf_gpio_pin_write(MCS_PIN_PWM0A, enable);
    nrf_gpio_pin_write(MCS_PIN_PWM0B, enable);
    nrf_gpio_pin_write(MCS_PIN_PWM2A, enable);
    nrf_gpio_pin_write(MCS_PIN_PWM2B, enable);
} 

static void update_steppers(bool release)
{
    uint8_t step_spi_cmd; 
    if(release)
    {
        step_spi_cmd = 0;
        //enable_pwms(false);
    }
    else
    {
        step_spi_cmd = sequence[m_pos_stepper_a] | sequence1[m_pos_stepper_b];
        //enable_pwms(true);
    }
    write_byte_spi(step_spi_cmd);
}

static void cont_step_timeout(void *p)
{
    step_ab(m_run_direction);
    if(m_stepper_on_period > 0)
    {
        app_timer_start(m_app_timer_step_delayed_release, APP_TIMER_TICKS(m_stepper_on_period), 0);
    }
}

static void step_delayed_release_timeout(void *p)
{
    pwm_stepper_duty_cycle_set(m_stepper_duty_cycle);
}

static void step_left_timeout(void *p)
{
    step_b(m_left_forward);
    if(m_left_interval_us <= m_right_interval_us)
    {
        pwm_stepper_duty_cycle_set(255);
        update_steppers(false);
        app_timer_start(m_app_timer_step_delayed_release, APP_TIMER_TICKS(m_stepper_on_period), 0);
    }
}

static void step_right_timeout(void *p)
{
    step_a(m_right_forward);
    if(m_right_interval_us < m_left_interval_us)
    {
        pwm_stepper_duty_cycle_set(255);
        update_steppers(false);
        app_timer_start(m_app_timer_step_delayed_release, APP_TIMER_TICKS(m_stepper_on_period), 0);
    }
}

void pwm_stepper_duty_cycle_set(uint8_t duty_cycle)
{
    duty_cycle = 255 - duty_cycle;
    m_pwm_stepper_seq_values.channel_0 = duty_cycle;
    m_pwm_stepper_seq_values.channel_1 = duty_cycle;
    m_pwm_stepper_seq_values.channel_2 = duty_cycle;
    m_pwm_stepper_seq_values.channel_3 = duty_cycle;
    
    nrf_drv_pwm_simple_playback(&m_pwm_stepper_control, &m_pwm_stepper_seq, 1, NRF_DRV_PWM_FLAG_LOOP);
}

void stepper_init(stepper_config_t *config)
{
    // Configure SPI interface
    nrf_drv_spi_config_t spi_config;
    spi_config.mosi_pin  = MCS_PIN_DIR_SER;
    spi_config.sck_pin   = MCS_PIN_DIR_CLK;
    spi_config.ss_pin    = MCS_PIN_DIR_LATCH;
    spi_config.miso_pin  = -1;
    spi_config.irq_priority = 7;
    spi_config.frequency = NRF_DRV_SPI_FREQ_125K;
    spi_config.mode      = NRF_DRV_SPI_MODE_0;
    spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_LSB_FIRST;
    ret_code = nrf_drv_spi_init(&m_spi_master_0, &spi_config, NULL, NULL);
    if (ret_code != NRF_SUCCESS)
    {
        printf("SPI INIT ERROR!\r\n");
        // Initialization failed. Take recovery action.
    }
           
    // Configure GPIO
    //nrf_gpio_cfg_output(MCS_PIN_PWM0A);   
    //nrf_gpio_cfg_output(MCS_PIN_PWM0B);
    nrf_gpio_cfg_output(MCS_PIN_PWM1A);   
    nrf_gpio_cfg_output(MCS_PIN_PWM1B);
    //nrf_gpio_cfg_output(MCS_PIN_PWM2A);
    //nrf_gpio_cfg_output(MCS_PIN_PWM2B);
    nrf_gpio_cfg_output(MCS_PIN_DIR_EN);
    //nrf_gpio_pin_set(MCS_PIN_PWM0A);
    //nrf_gpio_pin_set(MCS_PIN_PWM0B);
    nrf_gpio_pin_clear(MCS_PIN_PWM1A);
    nrf_gpio_pin_clear(MCS_PIN_PWM1B);
    //nrf_gpio_pin_set(MCS_PIN_PWM2A);
    //nrf_gpio_pin_set(MCS_PIN_PWM2B);
    nrf_gpio_pin_clear(MCS_PIN_DIR_EN);

    // PWM init
    nrf_drv_pwm_config_t pwm_config = 
    {
        .output_pins =
        {
            MCS_PIN_PWM0A,
            MCS_PIN_PWM0B,
            MCS_PIN_PWM2A,
            MCS_PIN_PWM2B,
        },
        .irq_priority = 7,
        .base_clock   = NRF_PWM_CLK_16MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = 255,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_TRIGGERED
    };
    nrf_drv_pwm_init(&m_pwm_stepper_control, &pwm_config, pwm_event_handler);
    pwm_stepper_duty_cycle_set(255);

    // Configure timers
    app_timer_create(&m_app_timer_step_delayed_release, APP_TIMER_MODE_SINGLE_SHOT, step_delayed_release_timeout);
    // Method B
    app_timer_create(&m_app_timer_step_left, APP_TIMER_MODE_REPEATED, step_left_timeout);
    app_timer_create(&m_app_timer_step_right, APP_TIMER_MODE_REPEATED, step_right_timeout);
}

void step_a(bool forward)
{
    m_pos_stepper_a += forward ? 3 : 1;
    m_pos_stepper_a %= 4;
}

void step_b(bool forward)
{
    m_pos_stepper_b += forward ? 1 : 3;
    m_pos_stepper_b %= 4;
}

void step_ab(step_direction_e direction)
{
    if(direction == DIR_NONE) update_steppers(true);
    else if(DIRECTION_VALID(direction))
    {
        m_pos_stepper_a += (direction == DIR_FORWARD || direction == DIR_TURN_LEFT) ? 3 : 1;
        m_pos_stepper_a %= 4;
        m_pos_stepper_b += (direction == DIR_FORWARD || direction == DIR_TURN_RIGHT) ? 1 : 3;
        m_pos_stepper_b %= 4;
        update_steppers(false);
    }
}

void step_release_all(void)
{
    update_steppers(true);
    app_timer_stop(m_app_timer_step_delayed_release);
}

char *dir_strings[] = {"NONE", "Forward", "Reverse", "Right turn", "Left turn", "Invalid!"};

void step_process_command(const uint8_t *ptr, uint32_t length)
{
    static uint32_t left_interval_previous = 0, right_interval_previous = 0;
    if(ptr[0] == 'A' && length >= 6)
    {
        // Check for a single command
        /*if(DIRECTION_VALID(ptr[1]))
        {
            printf("Single step: %s\r\n", dir_strings[ptr[1]]);
            step_ab(ptr[1]);
        }
        // Otherwise, check for a continuous command
        else if(DIRECTION_VALID(ptr[2]))
        {
            printf("Continuous: %s, %i, %i\r\n", dir_strings[ptr[2]], m_stepper_cont_interval, m_stepper_on_period);
            m_run_direction = ptr[2];
            m_stepper_cont_interval = ptr[3] << 8 | ptr[4];
            m_stepper_on_period = ptr[5] * m_stepper_cont_interval / 100;
            app_timer_start(m_app_timer_cont_step, APP_TIMER_TICKS(m_stepper_cont_interval), 0);
        }
        // If nothing is started, release the steppers
        else
        {
            printf("Stop\r\n");
            app_timer_stop(m_app_timer_cont_step);
            app_timer_stop(m_app_timer_step_left);
            app_timer_stop(m_app_timer_step_right);
            update_steppers(true);

        }*/
    }
    else if(ptr[0] == 'B' && length >= 9 )
    {
        for(int i = 0; i < 4; i++)
        {
            m_left_interval_us = (m_left_interval_us << 8) | ptr[1 + i];
            m_right_interval_us = (m_right_interval_us << 8) | ptr[5 + i];
        }
        if(m_left_interval_us & 0x80000000)
        {
            m_left_interval_us *= -1;
            m_left_forward = false;
        }else m_left_forward = true;
        if(m_right_interval_us & 0x80000000)
        {
            m_right_interval_us *= -1;
            m_right_forward = false;
        }else m_right_forward = true;

        if(m_stepper_duty_cycle != ptr[9])
        {
            m_stepper_duty_cycle = ptr[9];
        }

        if(m_left_interval_us != left_interval_previous || m_right_interval_us != right_interval_previous)
        {

            // Start or stop timers according to interval speeds
            app_timer_stop(m_app_timer_step_left);
            if(m_left_interval_us) app_timer_start(m_app_timer_step_left, APP_TIMER_TICKS((m_left_interval_us + 500) / 1000), 0);
            app_timer_stop(m_app_timer_step_right);
            if(m_right_interval_us) app_timer_start(m_app_timer_step_right, APP_TIMER_TICKS((m_right_interval_us + 500) / 1000), 0);
            left_interval_previous = m_left_interval_us;
            right_interval_previous = m_right_interval_us;
        }
        
        // If both motors are stopped, release steppers
        if(m_left_interval_us == 0 && m_right_interval_us == 0) update_steppers(false);
    }
}


