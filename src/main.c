#include "definitions.h"

esp_err_t init_isr(), create_tasks(); // Inicialización ISR's

static const char *TAG = "main"; // prints

void IRAM_ATTR isr_phase(void *arg)
{
    ph_count = gpio_get_level(HALL_PIN[0]) |
               (gpio_get_level(HALL_PIN[1]) << 1) |
               (gpio_get_level(HALL_PIN[2]) << 2);

    int64_t now = esp_timer_get_time();
    hall_dt = now - last_hall_time;
    last_hall_time = now;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (comm_task_handle != NULL)
    {
        vTaskNotifyGiveFromISR(comm_task_handle, &xHigherPriorityTaskWoken);
    }

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

void main_comm(void *arg)
{
    int local_ph = ph_count;
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        local_ph = ph_count;

        switch (local_ph)
        {
            case 4: set_duty(duty, 0, 0, duty, 0, 0);
            current_channel = ADC1_CHANNEL_0; break;

            case 6: set_duty(0, 0, 0, duty, duty, 0);
            current_channel = ADC1_CHANNEL_6; break;

            case 2: set_duty(0, duty, 0, 0, duty, 0);
            current_channel = ADC1_CHANNEL_6; break;

            case 3: set_duty(0, duty, duty, 0, 0, 0);
            current_channel = ADC1_CHANNEL_3; break;

            case 1: set_duty(0, 0, duty, 0, 0, duty);
            current_channel = ADC1_CHANNEL_3; break;

            case 5: set_duty(duty, 0, 0, 0, 0, duty);
            current_channel = ADC1_CHANNEL_0; break;

            default: set_duty(0, 0, 0, 0, 0, 0);break;
        }
    }
}

void current_control(void *arg)
{
    int64_t last_time_local = esp_timer_get_time();
    const int64_t current_interval = 100; // 100 us = 10 kHz para empezar

    while (1)
    {
        int64_t now = esp_timer_get_time();

        if ((now - last_time_local) >= current_interval)
        {
            last_time_local += current_interval;

            current_measurement = get_currents(duty);

            c_error = v_u - current_measurement;

            c_u = PID_calc(c_error,
                           PI_current[0],
                           PI_current[1],
                           current_interval / 1000000.0,
                           &integral_c,
                           true);

            if (c_u > 95.0f)
                c_u = 95.0f;
            else if (c_u < 0.0f)
                c_u = 0.0f;
                
        }
    }
}

void velocity_control(void *arg)
{
    int64_t last_time_local = esp_timer_get_time();
    const int64_t speed_interval = 10000; // 10000 us = 10 ms = 100 Hz

    while (1)
    {
        int64_t now = esp_timer_get_time();

        if ((now - last_time_local) >= speed_interval)
        {
            last_time_local += speed_interval;

            read_throttle(&adc_value);
            rpm = get_rpms();

            v_error = adc_value - rpm;

            v_u = PID_calc(v_error,
                                   PI_velocity[0],
                                   PI_velocity[1],
                                   speed_interval / 1000000.0,
                                   &integral_v,
                                   false);

            if (v_u > 5.0f)
                v_u = 5.0f;
            else if (v_u < 0.0f)
                v_u = 0.0f;

            ESP_LOGI(TAG, "Current: %f, RPM: %lld, duty: %f", current_measurement, rpm, duty);
        }
    }
}


void app_main()
{
    esp_task_wdt_deinit();
    for (int i = 0; i < 50; i++)
    {
        raw_A += adc1_get_raw(ADC1_CHANNEL_0);
        raw_B += adc1_get_raw(ADC1_CHANNEL_3);
        raw_C += adc1_get_raw(ADC1_CHANNEL_6);
    }
    raw_A = raw_A / 50;
    raw_B = raw_B / 50;
    raw_C = raw_C / 50;
    init_led();
    set_throttle();
    set_pwm();
    init_isr();
    create_tasks();
    ph_count = gpio_get_level(HALL_PIN[0]) | gpio_get_level(HALL_PIN[1]) << 1 | gpio_get_level(HALL_PIN[2]) << 2;
    // set_duty(0, 0, 0, 0, 0, 0); // Inicializa con duty 0
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
    BaseType_t ok;

    ok = xTaskCreatePinnedToCore(main_comm, "Commutation", 4096, NULL, 4, &comm_task_handle, 1);
    if (ok != pdPASS) ESP_LOGE("TASKS", "No se creó Commutation");

    ok = xTaskCreatePinnedToCore(current_control, "Current Control", 4096, NULL, 3, NULL, 1);
    if (ok != pdPASS) ESP_LOGE("TASKS", "No se creó Current Control");

    ok = xTaskCreatePinnedToCore(velocity_control, "Velocity Control", 4096, NULL, 2, NULL, 0);
    if (ok != pdPASS) ESP_LOGE("TASKS", "No se creó Velocity Control");

    return ESP_OK;
}