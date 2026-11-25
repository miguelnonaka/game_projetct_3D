#pragma once
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_err.h"
/* Pluviômetro de báscula (reed switch entre o sinal e o GND, pull-up da placa), lido por amostragem periódica. */
#define RAIN_GAUGE_DEBOUNCE_US_DEFAULT 50000U
esp_err_t rain_gauge_init(gpio_num_t pin, uint32_t debounce_us);
uint32_t rain_gauge_take_pulses(void);
