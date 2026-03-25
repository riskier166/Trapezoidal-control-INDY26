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
    while (true)
    {
        switch (ph_count)
        {
        case 4:
            set_duty(u, 0, 0, u, 0, 0); // Fase AH, BL
            // currentA = gen_current;
            break;
        case 6:
            set_duty(0, 0, 0, u, u, 0); // Fase BL, CH
            // currentC = gen_current;
            break;
        case 2:
            set_duty(0, u, 0, 0, u, 0); // Fase CH, AL
            // currentC = gen_current;
            break;
        case 3:
            set_duty(0, u, u, 0, 0, 0); // Fase BH, AL
            // currentB = gen_current;
            break;
        case 1:
            set_duty(0, 0, u, 0, 0, u); // Fase BH, CL
            // currentB = gen_current;
            break;
        case 5:
            set_duty(u, 0, 0, 0, 0, u); // Fase AH, CL
            // currentA = gen_current;
            break;
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
            //read_throttle(&adc_value);
            if (rpm_count > 0)
            {
                rpm = (rpm_count * 60000.0) / (TexCoeff);  
            }
            else
            {
                rpm = 0.0;
            }
            rpm_count = 0; // Reset RPM count every interval
            reference = REF_TEXAS; // Posteriormente cambiar por adc_value
            measurement = rpm;
            error = reference - measurement;
            u = PID_calc(error,PI_texas[0],PI_texas[1],interval);
            
            ESP_LOGI(TAG, "DUTY: %f, RPM's: %f Reference: %f\n", adc_value, rpm, reference);
        }
    }
}

void app_main()
{
    esp_task_wdt_deinit();
    init_led();set_throttle();set_pwm();
    init_isr();create_tasks();
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