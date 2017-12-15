#include "ardu_stepper_shield.h"
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_timer.h"

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

APP_TIMER_DEF(m_app_timer_cont_step);
APP_TIMER_DEF(m_app_timer_cont_step_release_delay);

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
        app_timer_start(m_app_timer_cont_step_release_delay, APP_TIMER_TICKS(m_stepper_on_period), 0);
    }
}

static void cont_step_release_timeout(void *p)
{
    update_steppers(true);
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

    /*nrf_gpio_cfg_output(MCS_PIN_DIR_CLK);
    nrf_gpio_cfg_output(MCS_PIN_DIR_LATCH);
    nrf_gpio_cfg_output(MCS_PIN_DIR_SER);
    nrf_gpio_pin_set(MCS_PIN_DIR_LATCH);*/

           
    // Configure GPIO
    nrf_gpio_cfg_output(MCS_PIN_PWM0A);   
    nrf_gpio_cfg_output(MCS_PIN_PWM0B);
    nrf_gpio_cfg_output(MCS_PIN_PWM1A);   
    nrf_gpio_cfg_output(MCS_PIN_PWM1B);
    nrf_gpio_cfg_output(MCS_PIN_PWM2A);
    nrf_gpio_cfg_output(MCS_PIN_PWM2B);
    nrf_gpio_cfg_output(MCS_PIN_DIR_EN);
    nrf_gpio_pin_set(MCS_PIN_PWM0A);
    nrf_gpio_pin_set(MCS_PIN_PWM0B);
    nrf_gpio_pin_clear(MCS_PIN_PWM1A);
    nrf_gpio_pin_clear(MCS_PIN_PWM1B);
    nrf_gpio_pin_set(MCS_PIN_PWM2A);
    nrf_gpio_pin_set(MCS_PIN_PWM2B);
    nrf_gpio_pin_clear(MCS_PIN_DIR_EN);

    // Configure timers
    app_timer_create(&m_app_timer_cont_step, APP_TIMER_MODE_REPEATED, cont_step_timeout);
    app_timer_create(&m_app_timer_cont_step_release_delay, APP_TIMER_MODE_SINGLE_SHOT, cont_step_release_timeout);

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
    app_timer_stop(m_app_timer_cont_step);
}

void step_cont_start(uint32_t interval_ms, bool forward)
{
    m_run_direction = forward;
    app_timer_start(m_app_timer_cont_step, APP_TIMER_TICKS(interval_ms), 0);
}

char *dir_strings[] = {"NONE", "Forward", "Reverse", "Right turn", "Left turn", "Invalid!"};
void step_process_command(const uint8_t *ptr, uint32_t length)
{
    if(length >= 6 && ptr[0] == 'A')
    {
        // Check for a single command
        if(DIRECTION_VALID(ptr[1]))
        {
            printf("Single step: %s\r\n", dir_strings[ptr[1]]);
            step_ab(ptr[1]);
            app_timer_stop(m_app_timer_cont_step);
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
            update_steppers(true);
            app_timer_stop(m_app_timer_cont_step);
        }
    }
}


