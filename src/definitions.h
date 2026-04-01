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

// polling & time dependances for control loop
int last_time = 0, interval = 100; // 1000us CURRENTinterval

// Velocity stuff
float rpm_measurement = 0; // RPM count

// Current stuff
static int last_valid = 2180;
static float current_filtered = 0.0;
const float alpha = 0.025; // Antes: 0.05
volatile float current_measurement = 0.0;
volatile adc1_channel_t current_channel;
// Currents
volatile float currentA, currentB, currentC, gen_current; // Current readings for each phase

// Control
float current_reference = 2, c_u = 0, c_error = 0.0;    // CURRENT control variables
float velocity_reference = 2, v_u = 0, v_error = 0.0;    // VELOCITY control variables
float PI_current[2] = {0.103672557, 160.221225}; // Coeficientes P:10.0, I:500.0
float PI_velocity[2] = {0.078709, 0.0049378}; // Coeficientes P:10.0, I:500.0
volatile float current, raw = 0, current_global = 0;
// PI
float prev_error = 0, integral = 0;

// Help Variables
volatile int ph_count = 0; // Hall sensors state
// PWM
volatile int adc_value = 0;
float duty = 40.0;       // Duty cycle vars
int deadTime_ticks = 64; // 64 ticks = 400 ns

// GPIO declarations
gpio_num_t LED_G = GPIO_NUM_16;                                    // Indicator LED pin
const int8_t adc_throttle = 4;                                     // Throttle ADC pin
const int8_t CH = 33, CL = 32, BH = 26, BL = 25, AH = 14, AL = 27; // PWM pins rectificados
const int8_t HALL_PIN[3] = {17, 18, 19};                           // Hall sensor pins

// RPM's calculation
int rpm_count = 0;
float rpm = 0; // Pshase count and RPM count
float TexCoeff = 240.0, RKV_Coeff = 1260.0;

float raw_A, raw_B, raw_C;


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

// Función para actualizar los duty cycles
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
    int raw = 0;

    esp_err_t ret = adc2_get_raw(
        ADC2_CHANNEL_0,
        ADC_WIDTH_BIT_12,
        &raw);

    if (ret == ESP_OK)
    {
        *value = (uint16_t)raw * 307.00 / 4095.00; // convert to percentage
    }

    return ret;
}

esp_err_t read_current()
{
    adc1_config_width(ADC_WIDTH_BIT_12);                        // Resolución de 12 bits
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

float get_currents()
{
    // promedio pa quitarle ruido a esta shit
    int sum = 0;
    for (int i = 0; i < 10; i++)
    {
        sum += adc1_get_raw(current_channel);
    }
    int raw = sum / 10;
    
    float Vout = 0.0;

    if (current_channel == ADC1_CHANNEL_0)
        Vout = ((raw-raw_A)/ (4095.0)) * 3.3;
    else if (current_channel == ADC1_CHANNEL_3)
        Vout = ((raw-raw_B)/ (4095.0)) * 3.3;
    else if (current_channel == ADC1_CHANNEL_6)
        Vout = ((raw-raw_C)/ (4095.0)) * 3.3;

    float Vsense = (Vout) / 20.0;
    float current = Vsense / 0.001;

    current = current * (duty / 100.0 + 0.133); // Compensación por duty cycle *15 funcionó chido*

    // filtro coqueto
    current_filtered = (alpha * current + (1 - alpha) * current_filtered);

    return fabs(current_filtered);
}

float get_rpms()
{
    rpm = ((rpm_count * 60000) / RKV_Coeff);
    //rpm = rpm_count * 4761.9;
    rpm_count = 0; // Reset RPM count every interval
    return rpm;
}

float PID_calc(float error, float Kp, float Ki, float dt)
{
    float U;

    // Proporcional
    float P = Kp * error;

    // Integral con anti-windup
    integral += error * dt;

    float I = Ki * integral;

    U = P + I;

    return U;
}

#endif // __DEFINITIONS_H__