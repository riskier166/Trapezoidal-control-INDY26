#include <stdio.h>
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "driver/mcpwm.h"

static const char *TAG = "main";

// GPIO declarations
const int8_t LED_G = 16;                                           // Indicator LED pin
const int8_t adc_throttle = 4;                                     // Throttle ADC pin
const int8_t CH = 33, CL = 32, BH = 26, BL = 25, AH = 14, AL = 27; // PWM pins rectificados
const int8_t HALL_PIN[3] = {17, 18, 19};                           // Hall sensor pins

// Help Variables
int adc_value = 0, led_state = 0, ph_count = 0, hall_a = 0, hall_b = 0, hall_c = 0, duty = 30;
// Main timer for loop
TimerHandle_t main_timer;
int count_timer = 50;

// Funciones de inicialización
esp_err_t init_led(), set_timer(), set_throttle(), set_pwm(), init_isr();

// Función ISR
void isr_phase(void *arg);

esp_err_t read_throttle(uint16_t *value) // ADC2 throttle reading function
{
    int raw = 0;

    esp_err_t ret = adc2_get_raw(
        ADC2_CHANNEL_0,
        ADC_WIDTH_BIT_12,
        &raw);

    if (ret == ESP_OK)
    {
        *value = (uint16_t)raw * 100 / 4095; // convert to percentage
    }

    return ret;
}

void vTimerCallback(TimerHandle_t xTimer) // timer callback
{
    // read_throttle(&adc_value); // convert to percentage
    //ESP_LOGI(TAG, "Value: %d", ph_count);
    switch (ph_count)
    {
    case 4:
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 0); // Fase CH
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, 0); // Fase CL
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, 0); // Fase BH
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_B, duty); // Fase BL
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_A, duty); // Fase AH
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_B, 0); // Fase AL
       break;
    case 6:
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, duty); // Fase CH
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, 0); // Fase CL
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, 0); // Fase BH
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_B, duty); // Fase BL
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_A, 0); // Fase AH
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_B, 0); // Fase AL
       break;
    case 2:
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, duty); // Fase CH
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, 0); // Fase CL
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, 0); // Fase BH
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_B, 0); // Fase BL
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_A, 0); // Fase AH
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_B, duty); // Fase AL
       break;
    case 3:
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 0); // Fase CH
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, 0); // Fase CL
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, duty); // Fase BH
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_B, 0); // Fase BL
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_A, 0); // Fase AH
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_B, duty); // Fase AL
       break;  
    case 1:
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 0); // Fase CH
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, duty); // Fase CL
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, duty); // Fase BH
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_B, 0); // Fase BL
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_A, 0); // Fase AH
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_B, 0); // Fase AL
       break;
    case 5:
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 0); // Fase CH
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, duty); // Fase CL
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, 0); // Fase BH
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_B, 0); // Fase BL
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_A, duty); // Fase AH
       mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_GEN_B, 0); // Fase AL
       break;
    }
    ESP_LOGI(TAG, "Phase: %d", ph_count);
}

void app_main()
{
    init_led();
    set_timer();
    set_throttle();
    set_pwm();
    init_isr();
    hall_a=gpio_get_level(HALL_PIN[0]);
    hall_b=gpio_get_level(HALL_PIN[1]) << 1;
    hall_c=gpio_get_level(HALL_PIN[2]) << 2;
    ph_count = hall_a | hall_b | hall_c;
}

esp_err_t set_timer()
{
    ESP_LOGI(TAG, "Timer initializing...");
    main_timer = xTimerCreate("main_timer",
                              pdMS_TO_TICKS(count_timer),
                              pdTRUE,
                              NULL,
                              vTimerCallback);
    if (main_timer == NULL)
    {
        ESP_LOGE(TAG, "Failed to create timer");
    }
    else
    {
        if (xTimerStart(main_timer, 0) != pdPASS)
        {
            ESP_LOGE(TAG, "Failed to start timer");
        }
    }

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

    uint32_t deadtime_ticks = 96; // 600 ns

    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0,
                          MCPWM_DEADTIME_BYPASS,
                          deadtime_ticks, deadtime_ticks);

    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1,
                          MCPWM_DEADTIME_BYPASS,
                          deadtime_ticks, deadtime_ticks);

    mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2,
                          MCPWM_DEADTIME_BYPASS,
                          deadtime_ticks, deadtime_ticks);

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

esp_err_t init_isr()
{

    gpio_config_t isr_config = {
        .pin_bit_mask = (1ULL << HALL_PIN[0]) |
                        (1ULL << HALL_PIN[1]) |
                        (1ULL << HALL_PIN[2]),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = 3};

    gpio_config(&isr_config);
    gpio_install_isr_service(0);

    for (int i = 0; i < 3; i++)
    {
        gpio_isr_handler_add(HALL_PIN[i], isr_phase, NULL);
    }

    return ESP_OK;
}

void isr_phase(void *arg)
{
    hall_a=gpio_get_level(HALL_PIN[0]);
    hall_b=gpio_get_level(HALL_PIN[1]) << 1;
    hall_c=gpio_get_level(HALL_PIN[2]) << 2;
    ph_count = hall_a | hall_b | hall_c;
}