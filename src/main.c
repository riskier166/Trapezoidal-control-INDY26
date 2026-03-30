#include "definitions.h"

esp_err_t init_isr(), create_tasks(); // Inicialización ISR's

static const char *TAG = "main"; // prints

void isr_phase(void *arg)
{
    ph_count = gpio_get_level(HALL_PIN[0]) | gpio_get_level(HALL_PIN[1]) << 1 | gpio_get_level(HALL_PIN[2]) << 2;
    rpm_count++;
}

void main_comm(void *arg)
{
    int last_ph = -1;

    while (true)
    {
        int local_ph = ph_count;

        if (local_ph != last_ph)
        {
            last_ph = local_ph;

            switch (local_ph)
            {
            case 4: // AH, BL → medir A
                set_duty(u, 0, 0, u, 0, 0);
                current_channel = ADC1_CHANNEL_0;
                break;

            case 6: // BL, CH → medir C
                set_duty(0, 0, 0, u, u, 0);
                current_channel = ADC1_CHANNEL_6;
                break;

            case 2: // CH, AL → medir C
                set_duty(0, u, 0, 0, u, 0);
                current_channel = ADC1_CHANNEL_6;
                break;

            case 3: // BH, AL → medir B
                set_duty(0, u, u, 0, 0, 0);
                current_channel = ADC1_CHANNEL_3;
                break;

            case 1: // BH, CL → medir B
                set_duty(0, 0, u, 0, 0, u);
                current_channel = ADC1_CHANNEL_3;
                break;

            case 5: // AH, CL → medir A
                set_duty(u, 0, 0, 0, 0, u);
                current_channel = ADC1_CHANNEL_0;
                break;
            }
        }
    }
}

void control_read(void *arg)
{
    while (1)
    {
        int64_t current_time = esp_timer_get_time();

        if ((current_time - last_time) >= interval)
        {
            last_time = current_time;

            float dt = interval / 1000000.0;

            measurement = get_currents();

            error = current_reference - measurement;

            u = PID_calc(error, PI_R100[0], PI_R100[1], dt);

            // saturación
            if (u > 95.0)
                u = 95.0;
            if (u < 5.0)
                u = 5.0;

            ESP_LOGE(TAG, "Current: %f", get_currents());
        }
    }
}

void app_main()
{
    esp_task_wdt_deinit();
    init_led();
    set_throttle();
    set_pwm();
    init_isr();
    create_tasks();
    ph_count = gpio_get_level(HALL_PIN[0]) | gpio_get_level(HALL_PIN[1]) << 1 | gpio_get_level(HALL_PIN[2]) << 2;
    set_duty(0, 0, 0, 0, 0, 0); // Inicializa con duty 0
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

esp_err_t create_tasks()
{
    static uint8_t ucParameterToPass;
    TaskHandle_t xHandle = NULL;
    xTaskCreate(main_comm,
                "Commutation",
                4096,
                &ucParameterToPass,
                1,
                &xHandle);
    xTaskCreate(control_read,
                "Control Read",
                4096,
                &ucParameterToPass,
                2,
                &xHandle);
    return ESP_OK;
}