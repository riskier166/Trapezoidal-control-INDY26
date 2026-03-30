#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__

#include <stdio.h>
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "driver/mcpwm.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "math.h"

//Control
#define REF_R100 0.1299*(adc_value*adc_value*adc_value)-15.605*(adc_value*adc_value)+649.08*adc_value-5931.8;
#define REF_TEXAS -0.4792*(adc_value*adc_value)+110.01*(adc_value)-1276.4;
float reference = 0.0, measurement = 0.0, u = 0, error = 0.0; // control variables 
float PI_texas [2] = {13.143269,22783.72239}; // Coeficientes P:13.143269, I:22783.72239
volatile float current, raw = 0, current_global = 0, current_channel;
float prev_error=0, integral=0;

// GPIO declarations
gpio_num_t LED_G = GPIO_NUM_16; // Indicator LED pin
const int8_t adc_throttle = 4; // Throttle ADC pin
const int8_t CH = 33, CL = 32, BH = 26, BL = 25, AH = 14, AL = 27; // PWM pins rectificados
const int8_t HALL_PIN[3] = {17, 18, 19}; // Hall sensor pins

// Help Variables
volatile int ph_count = 0; // Hall sensors state

// RPM's calculation 
int rpm_count = 0;
float rpm = 0; // Pshase count and RPM count
float TexCoeff = 240.0, RKV_Coeff = 1260.0;

// PWM
float adc_value = 0.0; // ADC Throttle
float duty = 40.0; // Duty cycle
int deadTime_ticks = 64; // 64 ticks = 400 ns

//Currents
volatile float currentA, currentB, currentC, gen_current; // Current readings for each phase

//polling 
int last_time = 0, interval = 1000; // 1 ms interval, 

esp_err_t set_pwm()
{
    mcpwm_config_t pwm_config = {
        .frequency = 20000, // 20 kHz
        .cmpr_a = 0,        // Duty inicial 0 para todos
        .cmpr_b = 0,
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_DOWN_COUNTER // Center aligned
    };

    // TIMER 0 (FASE C)
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, CH);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, CL);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

    // TIMER 1 (FASE B)
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, BH);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, BL);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);

    // TIMER 2 (FASE A)
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, AH);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, AL);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);

    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0,
                          MCPWM_DEADTIME_BYPASS,
                          deadTime_ticks, deadTime_ticks);

    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1,
                          MCPWM_DEADTIME_BYPASS,
                          deadTime_ticks, deadTime_ticks);

    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2,
                          MCPWM_DEADTIME_BYPASS,
                          deadTime_ticks, deadTime_ticks);

    mcpwm_sync_config_t sync_conf = {
        .sync_sig = MCPWM_SELECT_TIMER0_SYNC,
        .timer_val = 0,
        .count_direction = MCPWM_TIMER_DIRECTION_UP};

    mcpwm_sync_configure(MCPWM_UNIT_0, MCPWM_TIMER_1, &sync_conf);
    mcpwm_sync_configure(MCPWM_UNIT_0, MCPWM_TIMER_2, &sync_conf);

    mcpwm_set_timer_sync_output(MCPWM_UNIT_0,
                                MCPWM_TIMER_0,
                                MCPWM_SWSYNC_SOURCE_TEZ);
    return ESP_OK;
}

//Función para actualizar los duty cycles
void set_duty(float AH, float AL, float BH, float BL, float CH, float CL)
{
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_A, AH); // Fase AH
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_B, AL); // Fase AL
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, BH); // Fase BH
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_B, BL); // Fase BL
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, CH); // Fase CH
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, CL); // Fase CL
}

// ADC2 throttle reading function
esp_err_t read_throttle(uint16_t *value) 
{
    float raw = 0;

    esp_err_t ret = adc2_get_raw(
        ADC2_CHANNEL_0,
        ADC_WIDTH_BIT_12,
        &raw);

    if (ret == ESP_OK)
    {
        *value = (float)raw * 100.00 / 4095.00; // convert to percentage
    }

    return ret;
}

esp_err_t read_current()
{
    adc1_config_width(ADC_WIDTH_BIT_12); // Resolución de 12 bits
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_12); // GPIO36, fase A
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_12); // GPIO39, fase B
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_12); // GPIO34, fase C
    return ESP_OK;
}

esp_err_t init_led()
{
    gpio_reset_pin(LED_G);
    gpio_set_direction(LED_G, GPIO_MODE_OUTPUT);
    return ESP_OK;
}

esp_err_t set_throttle(void)
{
    esp_err_t ret = adc2_config_channel_atten(
        ADC2_CHANNEL_0,
        ADC_ATTEN_DB_11 // hasta ~3.3V
    );
    return ret;
}

float PID_calc(float error, float Kp, float Ki, float dt)
{
    float U;

    // Proporcional
    float P = Kp * error;

    // Integral con anti-windup
    integral += error * dt;

    // Clamp de integral
    if (integral > 5.0) integral = 5.0;
    if (integral < -5.0) integral = -5.0;

    float I = Ki * integral;

    U = P + I;

    return U;
}

#endif // __DEFINITIONS_H__


