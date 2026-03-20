#include "definitions.h"

esp_err_t init_isr(), set_timer(); // Inicialización ISR's

static const char *TAG = "main"; // prints

void isr_phase(void *arg)
{
    ph_count = gpio_get_level(HALL_PIN[0])
            | gpio_get_level(HALL_PIN[1]) << 1 
            | gpio_get_level(HALL_PIN[2]) << 2;
}

void app_main()
{
    init_led(); 
    set_timer(); 
    set_throttle(); set_pwm(); init_isr();
    ph_count = gpio_get_level(HALL_PIN[0]) 
            | gpio_get_level(HALL_PIN[1]) << 1 
            | gpio_get_level(HALL_PIN[2]) << 2;
    set_duty(0,0,0,0,0,0); // Inicializa con duty 0

    while (true)
    {
        switch (ph_count)
        {
        case 4:
            set_duty(duty, 0, 0, duty, 0, 0); // Fase AH, BL
            // currentA = gen_current;
            break;
        case 6:
            set_duty(0, 0, 0, duty, duty, 0); // Fase BL, CH
            // currentC = gen_current;
            break;
        case 2:
            set_duty(0, duty, 0, 0, duty, 0); // Fase CH, AL
            // currentC = gen_current;
            break;
        case 3:
            set_duty(0, duty, duty, 0, 0, 0); // Fase BH, AL
            // currentB = gen_current;
            break;
        case 1:
            set_duty(0, 0, duty, 0, 0, duty); // Fase BH, CL
            // currentB = gen_current;
            break;
        case 5:
            set_duty(duty, 0, 0, 0, 0, duty); // Fase AH, CL
            // currentA = gen_current;
            break;
        }
    }
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