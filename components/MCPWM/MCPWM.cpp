// Code from examples, Espressif:
// https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/mcpwm.html

#include "MCPWM.hpp"
#include "esp_check.h"
#include <math.h>

static const char* TAG = "Mcpwm";


esp_err_t Mcpwm::init(const Config& cfg) {
    if (_initialised) return ESP_ERR_INVALID_STATE;

    _cfg = cfg;
    // centre-aligned PWM in UP_DOWN mode
    _period_tics = _cfg.timer_resolution_hz / (2 * _cfg.pwm_freq_hz);
    if (_period_tics < 20) {
        ESP_LOGE(TAG, "Period too small: %lu ticks", _period_tics);
        return ESP_ERR_INVALID_ARG;
    }

    mcpwm_timer_config_t timer_cfg = {};
    timer_cfg.group_id = _cfg.group_id;
    timer_cfg.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_cfg.resolution_hz = _cfg.timer_resolution_hz;
    timer_cfg.period_ticks = _period_tics;
    timer_cfg.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN;
    
    ESP_RETURN_ON_ERROR(mcpwm_new_timer(&timer_cfg, &_timer), TAG, "create timer failed");

    // 3 phases
    ESP_RETURN_ON_ERROR(init_single_phase(phase_a, _cfg.pwm_a_gpio), TAG, "phase A failed");
    ESP_RETURN_ON_ERROR(init_single_phase(phase_b, _cfg.pwm_b_gpio), TAG, "phase B failed");
    ESP_RETURN_ON_ERROR(init_single_phase(phase_c, _cfg.pwm_c_gpio), TAG, "phase C failed");

    ESP_RETURN_ON_ERROR(mcpwm_timer_enable(_timer), TAG, "timer enable failed");
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(_timer, MCPWM_TIMER_START_NO_STOP), TAG, "timer start failed");

    _initialised = true;
    _enabled = false;

    if (_cfg.n_sleep_gpio != GPIO_NUM_NC) { gpio_set_level(_cfg.n_sleep_gpio, 0); }

    ESP_LOGI(TAG, "Init OK: freq=%lu Hz, period=%lu ticks, GPIOs A=%d B=%d C=%d", _cfg.pwm_freq_hz, _period_tics, _cfg.pwm_a_gpio, _cfg.pwm_b_gpio, _cfg.pwm_c_gpio);
    
    return ESP_OK;
}

esp_err_t Mcpwm::init_single_phase(Phase& phase, gpio_num_t pwm_gpio) {
    mcpwm_operator_config_t oper_cfg = {};
    oper_cfg.group_id = _cfg.group_id;
    ESP_RETURN_ON_ERROR(mcpwm_new_operator(&oper_cfg, &phase.oper), TAG, "operator failed");
    
    ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(phase.oper, _timer), TAG, "connect timer failed");

    // comp with shadow update on both zero and peak
    mcpwm_comparator_config_t cmp_cfg = {};
    cmp_cfg.flags.update_cmp_on_tez = true;
    cmp_cfg.flags.update_cmp_on_tep = true;
    ESP_RETURN_ON_ERROR(mcpwm_new_comparator(phase.oper, &cmp_cfg, &phase.cmpr), TAG, "comparator failed");

    mcpwm_generator_config_t gen_cfg = {};
    gen_cfg.gen_gpio_num = pwm_gpio;
    ESP_RETURN_ON_ERROR(mcpwm_new_generator(phase.oper, &gen_cfg, &phase.gen), TAG, "generator failed");

    // symmetric centre-aligned PWM:
    // - on UP compare: set HIGH
    // - on DOWN compare: set LOW
    ESP_RETURN_ON_ERROR(mcpwm_generator_set_actions_on_compare_event(phase.gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, phase.cmpr, MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, phase.cmpr, MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END()), TAG, "compare actions failed");

    // set a safe init cmp value away from 0 and peak
    uint32_t safe_init_cmp = safe_compare_from_normalised(0.5f);
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(phase.cmpr, safe_init_cmp), TAG, "cmp init failed");
    
    return ESP_OK;
}

esp_err_t Mcpwm::set_phase_voltages(float va, float vb, float vc) {
    if (!_initialised || !_enabled) return ESP_ERR_INVALID_STATE;

    auto set_duty = [&](Phase& phase, float voltage) -> esp_err_t {
        auto bound = fminf(fmaxf(voltage, -1.0f), 1.0f);
        auto normalised = (bound + 1.0f) * 0.5f;
        uint32_t cmp = safe_compare_from_normalised(normalised);
        return mcpwm_comparator_set_compare_value(phase.cmpr, cmp);
    };

    ESP_RETURN_ON_ERROR(set_duty(phase_a, va), TAG, "phase A failed");
    ESP_RETURN_ON_ERROR(set_duty(phase_b, vb), TAG, "phase B failed");
    ESP_RETURN_ON_ERROR(set_duty(phase_c, vc), TAG, "phase C failed");

    ESP_LOGD(TAG, "Set A=%.3f B=%.3f C=%.3f", va, vb, vc);
    return ESP_OK;
}

esp_err_t Mcpwm::enable() {
    if (!_initialised) { return ESP_ERR_INVALID_STATE; }

    if (_cfg.n_sleep_gpio != GPIO_NUM_NC) { gpio_set_level(_cfg.n_sleep_gpio, 1); }

    _enabled = true;

    ESP_RETURN_ON_ERROR(set_phase_voltages(0.0f, 0.0f, 0.0f), TAG, "set neutral duty in enable failed");

    return ESP_OK;
}

esp_err_t Mcpwm::disable() {
    if (!_initialised) return ESP_ERR_INVALID_STATE;

    _enabled = false;

    if (_cfg.n_sleep_gpio != GPIO_NUM_NC) { gpio_set_level(_cfg.n_sleep_gpio, 0); }
    
    return ESP_OK;
}

uint32_t Mcpwm::safe_compare_from_normalised(float normalised) const {
    float n = fminf(fmaxf(normalised, 0.0f), 1.0f);

    uint32_t peak = _period_tics / 2;
    if (peak <= 2) return 1;

    uint32_t cmp = (uint32_t)lroundf(n * (float)peak);

    if (cmp == 0) cmp = 1;
    if (cmp >= peak) cmp = peak - 1;

    return cmp;
}
