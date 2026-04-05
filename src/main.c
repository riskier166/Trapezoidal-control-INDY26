#include "definitions.h"

esp_err_t init_isr(), create_tasks(); // Inicialización ISR's

static const char *TAG = "main"; // prints

void isr_phase(void *arg)
{
    ph_count = gpio_get_level(HALL_PIN[0]) | gpio_get_level(HALL_PIN[1]) << 1 | gpio_get_level(HALL_PIN[2]) << 2;
    rpm_count++;
    int64_t now = esp_timer_get_time();
    hall_dt = now - last_hall_time;
    last_hall_time = now;
}

void main_comm(void *arg)
{
    int last_ph = -1;

    // Secuencia de arranque open-loop
    const int startup_seq[6] = {5, 4, 6, 2, 3, 1};
    int startup_idx = 0;

    // Control de tiempo para startup
    int64_t last_startup_step = 0;
    bool aligned = false;

    while (true)
    {
        int64_t now = esp_timer_get_time();
        int local_rpm = rpm;
        int local_ph = ph_count;
        if (local_rpm < 20 && adc_value > 20) // umbral de arranque
        {
            //Alineación inicial 
            if (!aligned)
            {
                commutate(local_ph, duty); // Commutate según el estado actual de los sensores Hall

                last_startup_step = now;
                aligned = true;

                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            if ((now - last_startup_step) >= 20000) // 40 ms por paso, ajustable
            {
                last_startup_step = now;

                commutate(startup_seq[startup_idx], duty); // Commutate al siguiente estado de la secuencia de arranque
                startup_idx++;
                if (startup_idx >= 6)
                    startup_idx = 0;
            }
        }
        else
        {
            aligned = false; // reinicia startup para la próxima vez

            if (local_ph != last_ph)
            {
                last_ph = local_ph;

                commutate(local_ph, c_u); // Commutate según el estado actual de los sensores Hall
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void current_control(void *arg)
{
    while (1)
    {
        int64_t current_time = esp_timer_get_time();

        if ((current_time - last_time) >= interval)
        {

            last_time = current_time;

            // Lecturas
            read_throttle(&adc_value);
            rpm = get_rpms();
            current_measurement = get_currents();

            // Velocity control applied
            v_error = adc_value - rpm;
            v_u = PID_calc(v_error, PI_velocity[0], PI_velocity[1], interval / 1000000.0, &integral_v, false);

            // Current control applied
            c_error = v_u - current_measurement;
            c_u = PID_calc(c_error, PI_current[0], PI_current[1], interval / 1000000.0, &integral_c, true);
            // Saturación V_U
            if (c_u > 95.0)
                c_u = 95.0;
            else if (c_u < 0.0)
                c_u = 0.0;

            ESP_LOGW(TAG, "current: %f, rpm: %lld, DUTY: %f, Desired rpm's: %d\n", current_measurement, rpm, c_u, adc_value);
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
    xTaskCreate(current_control,
                "Current Control",
                4096,
                &ucParameterToPass,
                2,
                &xHandle);
    return ESP_OK;
}