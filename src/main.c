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
                set_duty(duty, 0, 0, duty, 0, 0);
                current_channel = ADC1_CHANNEL_0;
                break;

            case 6: // BL, CH → medir C
                set_duty(0, 0, 0, duty, duty, 0);
                current_channel = ADC1_CHANNEL_6;
                break;

            case 2: // CH, AL → medir C
                set_duty(0, duty, 0, 0, duty, 0);
                current_channel = ADC1_CHANNEL_6;
                break;

            case 3: // BH, AL → medir B
                set_duty(0, duty, duty, 0, 0, 0);
                current_channel = ADC1_CHANNEL_3;
                break;

            case 1: // BH, CL → medir B
                set_duty(0, 0, duty, 0, 0, duty);
                current_channel = ADC1_CHANNEL_3;
                break;

            case 5: // AH, CL → medir A
                set_duty(duty, 0, 0, 0, 0, duty);
                current_channel = ADC1_CHANNEL_0;
                break;
            }
        }
    }
}

void control_read(void *arg)
{
    static int last_valid = 2180;
    static float current_filtered = 0.0;
    const float alpha = 0.2;

    while (1)
    {
        int64_t current_time = esp_timer_get_time();

        if ((current_time - last_time) >= interval)
        {
            last_time = current_time;

            // pequeño delay para evitar switching noise
            esp_rom_delay_us(2);

            // oversampling
            int sum = 0;
            for (int i = 0; i < 4; i++)  // puedes bajar a 4 para aligerar
            {
                sum += adc1_get_raw(current_channel);
            }
            int raw = sum / 4;

            // clamp
            if (raw < 1800 || raw > 2600)
            {
                raw = last_valid;
            }
            else
            {
                last_valid = raw;
            }

            float Vout = (raw / 4095.0) * 3.3;
            float Vsense = (Vout - 1.75) / 20.0;
            float current = Vsense / 0.001;

            // filtro
            current_filtered = alpha * current + (1 - alpha) * current_filtered;

            current_global = fabs(current_filtered);

            ESP_LOGE(TAG, "Current: %f", current_global);
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