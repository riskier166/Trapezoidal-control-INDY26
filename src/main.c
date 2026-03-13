#include "definitions.h"

esp_err_t init_isr(); // Funciones de inicialización
void isr_phase(void *arg); // Función ISR

static const char *TAG = "main"; //prints

void vTimerCallback(TimerHandle_t xTimer) // timer callback
{
    // read_throttle(&adc_value); // convert to percentage
    switch (ph_count)
    {
    case 4:
        set_duty(duty, 0, 0, duty, 0, 0); // Fase AH, BL
        break;
    case 6:
        set_duty(0, 0, 0, duty, duty, 0); // Fase BL, CH
        break;
    case 2:
        set_duty(0, duty, 0, 0, duty, 0); // Fase CH
        break;
    case 3:
        set_duty(0, duty, duty, 0, 0, 0); // Fase BH
        break;
    case 1:
        set_duty(0, 0, duty, 0, 0, duty); // Fase AH, CL
        break;
    case 5:
        set_duty(duty, 0, 0, 0, 0, duty); // Fase AH, CL
        break;
    }
    //ESP_LOGI(TAG, "Phase: %d", ph_count);
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

void app_main()
{
    init_led(); set_timer(); set_throttle(); set_pwm(); init_isr();
    hall_a = gpio_get_level(HALL_PIN[0]);
    hall_b = gpio_get_level(HALL_PIN[1]) << 1;
    hall_c = gpio_get_level(HALL_PIN[2]) << 2;
    ph_count = hall_a | hall_b | hall_c;
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
    hall_a = gpio_get_level(HALL_PIN[0]);
    hall_b = gpio_get_level(HALL_PIN[1]) << 1;
    hall_c = gpio_get_level(HALL_PIN[2]) << 2;
    ph_count = hall_a | hall_b | hall_c;
}