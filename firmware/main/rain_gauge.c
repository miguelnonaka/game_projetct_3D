/*
 * Pluviômetro de báscula em um único pino, lido por amostragem periódica (esp_timer a
 * cada 2 ms), sem interrupção de GPIO. Só o pino passado a rain_gauge_init é vigiado.
 *
 * Ligação: contato (reed switch) entre o sinal e o GND, módulo de 2 fios. O fechamento do
 * contato puxa o sinal para GND, ou seja, para nível baixo.
 *
 * Nesta placa o GPIO34 não tem pull-up (é entrada pura, sem pulls internos) e não há
 * resistor externo. Por isso o sinal se comporta assim:
 *   - contato aberto: o pino flutua e lê 0/1 alternando com ruído (dezenas de bordas
 *     altas por segundo);
 *   - contato fechado (a báscula passando pelo meio): o pino fica firme em 0, sem ruído.
 *
 * Uma basculada é, portanto, um trecho contínuo em nível baixo de pelo menos
 * RAIN_GAUGE_CLOSED_MIN_MS (padrão 30 ms: na bancada o ruído com o contato aberto não passou de 12 ms em nível baixo, e basculadas reais ficaram entre ~30 e ~130 ms). Um único nível alto encerra o trecho: com o
 * contato fechado a linha é aterrada e não sobe.
 *
 * Cada leitura registra no log o maior trecho em nível baixo visto desde a leitura anterior.
 * Use esse valor para ajustar o limite com -DRAIN_GAUGE_CLOSED_MIN_MS=... sem alterar o código.
 *
 * Correção definitiva: resistor de pull-up de 10 kΩ entre o sinal e o 3V3. Com ele, o contato
 * aberto deixa a linha em 1 estável e o contato fechado a leva a 0; a mesma lógica continua
 * valendo.
 *
 * Motivo da amostragem: no ESP32 (errata dos GPIO36/GPIO39), com Wi-Fi em modem sleep ou com
 * o ADC ativo, a entrada recebe pulsos curtos e espúrios, e um cabo longo capta ruído. Com
 * interrupção em cada borda, cada um desses pulsos virava contagem. Aqui o nível é amostrado
 * a cada 2 ms e a contagem exige um trecho baixo contínuo.
 */
#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#include "rain_gauge.h"

#define RAIN_GAUGE_SAMPLE_PERIOD_US 2000U
#define RAIN_GAUGE_SAMPLE_PERIOD_MS (RAIN_GAUGE_SAMPLE_PERIOD_US / 1000U)

#ifndef RAIN_GAUGE_CLOSED_MIN_MS
#define RAIN_GAUGE_CLOSED_MIN_MS 30U
#endif

/* Saturação do contador de amostras: com período de 2 ms, o produto ainda cabe em uint32_t. */
#define RAIN_GAUGE_LOW_RUN_MAX (UINT32_MAX / 2U)

static const char *TAG = "rain_gauge";
static gpio_num_t s_pin = GPIO_NUM_NC;
static uint32_t s_low_run_samples;
static bool s_run_counted;
static uint32_t s_max_low_run_samples;
static uint32_t s_pulse_count;
static int64_t s_last_pulse_us = INT64_MIN / 2;
static uint32_t s_debounce_us;
static bool s_initialized;
static esp_timer_handle_t s_timer;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static void rain_gauge_count_tip(void)
{
    const int64_t now = esp_timer_get_time();
    /* Descarta um novo fechamento dentro da janela de debounce. */
    if (now - s_last_pulse_us >= (int64_t)s_debounce_us) {
        portENTER_CRITICAL(&s_mux);
        s_pulse_count++;
        portEXIT_CRITICAL(&s_mux);
        s_last_pulse_us = now;
    }
}

static void rain_gauge_sample(void *arg)
{
    (void)arg;

    const int level = gpio_get_level(s_pin);

    if (level == 0) {
        /* Nível baixo: o trecho cresce. Satura para não estourar o contador. */
        if (s_low_run_samples < RAIN_GAUGE_LOW_RUN_MAX) {
            s_low_run_samples++;
        }
        portENTER_CRITICAL(&s_mux);
        if (s_low_run_samples > s_max_low_run_samples) {
            s_max_low_run_samples = s_low_run_samples;
        }
        portEXIT_CRITICAL(&s_mux);

        /* Trecho baixo contínuo e longo o bastante: conta uma vez por trecho. */
        if (!s_run_counted &&
            s_low_run_samples * RAIN_GAUGE_SAMPLE_PERIOD_MS >= RAIN_GAUGE_CLOSED_MIN_MS) {
            rain_gauge_count_tip();
            s_run_counted = true;
        }
    } else {
        /* Nível alto: fim do trecho (contato aberto ou ruído). O próximo trecho baixo é novo. */
        s_low_run_samples = 0;
        s_run_counted = false;
    }
}

esp_err_t rain_gauge_init(gpio_num_t pin, uint32_t debounce_us)
{
    if (s_initialized) return ESP_ERR_INVALID_STATE;
    if (pin < GPIO_NUM_0 || pin >= GPIO_NUM_MAX ||
        (pin >= GPIO_NUM_6 && pin <= GPIO_NUM_11) ||
        debounce_us < 1000U || debounce_us > 1000000U) {
        return ESP_ERR_INVALID_ARG;
    }

    /* GPIO34–39 não têm pull interno: sem pull-up externo, o contato aberto deixa a linha
     * flutuando (ver o cabeçalho deste arquivo). */
    const gpio_config_t pin_config = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&pin_config), TAG,
                        "falha ao configurar pluviômetro");

    /* Estado pronto antes de iniciar o temporizador. */
    s_pin = pin;
    s_low_run_samples = 0;
    s_max_low_run_samples = 0;
    s_debounce_us = debounce_us;
    s_last_pulse_us = INT64_MIN / 2;
    s_pulse_count = 0;

    const int level = gpio_get_level(pin);
    /* Contato já fechado no boot: o trecho em curso não conta; só conta um trecho novo,
     * depois que o contato abrir e fechar de novo. */
    s_run_counted = (level == 0);
    ESP_LOGI(TAG, "pluviômetro no GPIO%d; em repouso=%d; leitura a cada %u ms; basculada = nível baixo contínuo por >= %u ms",
             (int)pin, level, (unsigned)RAIN_GAUGE_SAMPLE_PERIOD_MS, (unsigned)RAIN_GAUGE_CLOSED_MIN_MS);

    const esp_timer_create_args_t timer_args = {
        .callback = rain_gauge_sample,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "rain_gauge",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&timer_args, &s_timer), TAG,
                        "falha ao criar temporizador de amostragem do pluviômetro");
    esp_err_t error = esp_timer_start_periodic(s_timer, RAIN_GAUGE_SAMPLE_PERIOD_US);
    if (error != ESP_OK) {
        esp_timer_delete(s_timer);
        s_timer = NULL;
        ESP_LOGE(TAG, "falha ao iniciar temporizador de amostragem: %s", esp_err_to_name(error));
        return error;
    }
    s_initialized = true;
    return ESP_OK;
}

uint32_t rain_gauge_take_pulses(void)
{
    if (!s_initialized) return 0;
    portENTER_CRITICAL(&s_mux);
    const uint32_t pulses = s_pulse_count;
    const uint32_t max_samples = s_max_low_run_samples;
    s_pulse_count = 0;
    s_max_low_run_samples = 0;
    portEXIT_CRITICAL(&s_mux);

    ESP_LOGI(TAG, "pluviômetro: basculadas=%u; maior trecho em nível baixo=%u ms (mínimo para contar: %u ms)",
             (unsigned)pulses, (unsigned)(max_samples * RAIN_GAUGE_SAMPLE_PERIOD_MS),
             (unsigned)RAIN_GAUGE_CLOSED_MIN_MS);
    return pulses;
}
