
#include "MCPWM.hpp"
#include "esp_check.h"

static const char* TAG = "Mcpwm";


esp_err_t Mcpwm::init(const Config& cfg) {
    if (_initialised) return ESP_ERR_INVALID_STATE;

    _cfg = cfg;
    _period_tics = _cfg.timer_resolution_hz / _cfg.pwm_freq_hz;
    if (_period_tics < 10) {
        ESP_LOGE(TAG, "Period too small: %lu ticks", _period_tics);
        return ESP_ERR_INVALID_ARG;
    }

    // timer
    mcpwm_timer_config_t timer_cfg = {};
    timer_cfg.group_id = _cfg.group_id;
    timer_cfg.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_cfg.resolution_hz = _cfg.timer_resolution_hz;
    timer_cfg.period_ticks = _period_tics;
    timer_cfg.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    
    ESP_RETURN_ON_ERROR(mcpwm_new_timer(&timer_cfg, &_timer), TAG, "create timer failed");

    // 3 phases
    ESP_RETURN_ON_ERROR(init_single_phase(phase_a, _cfg.pwm_a_gpio), TAG, "phase A failed");
    ESP_RETURN_ON_ERROR(init_single_phase(phase_b, _cfg.pwm_b_gpio), TAG, "phase B failed");
    ESP_RETURN_ON_ERROR(init_single_phase(phase_c, _cfg.pwm_c_gpio), TAG, "phase C failed");

    ESP_RETURN_ON_ERROR(mcpwm_timer_enable(_timer), TAG, "timer enable failed");
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(_timer, MCPWM_TIMER_START_NO_STOP), TAG, "timer start failed");

    _initialised = true;
    ESP_RETURN_ON_ERROR(all_off(), TAG, "init all off failed");

    ESP_LOGI(TAG, "Init OK: freq=%lu Hz, period=%lu ticks, GPIOs A=%d B=%d C=%d", _cfg.pwm_freq_hz, _period_tics, _cfg.pwm_a_gpio, _cfg.pwm_b_gpio, _cfg.pwm_c_gpio);
    
    return ESP_OK;
}

esp_err_t Mcpwm::init_single_phase(Phase& phase, gpio_num_t pwm_gpio) {
    mcpwm_operator_config_t oper_cfg = {};
    oper_cfg.group_id = _cfg.group_id;
    ESP_RETURN_ON_ERROR(mcpwm_new_operator(&oper_cfg, &phase.oper), TAG, "operator failed");

    ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(phase.oper, _timer), TAG, "connect timer failed");

    mcpwm_comparator_config_t cmp_cfg = {};
    cmp_cfg.flags.update_cmp_on_tez = true;
    ESP_RETURN_ON_ERROR(mcpwm_new_comparator(phase.oper, &cmp_cfg, &phase.cmpr), TAG, "comparator failed");

    mcpwm_generator_config_t gen_cfg = {};
    gen_cfg.gen_gpio_num = pwm_gpio;
    ESP_RETURN_ON_ERROR(mcpwm_new_generator(phase.oper, &gen_cfg, &phase.gen), TAG, "gen failed");

    ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_timer_event(phase.gen, MCPWM_GEN_TIMER_EVENT_ACTION
        (MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)), TAG, "timer action failed");

    ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_compare_event(phase.gen, MCPWM_GEN_COMPARE_EVENT_ACTION
        (MCPWM_TIMER_DIRECTION_UP, phase.cmpr, MCPWM_GEN_ACTION_LOW)), TAG, "compare action failed");

    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(phase.cmpr, 0), TAG, "cmp init failed");
    
    return ESP_OK;
}

esp_err_t Mcpwm::set_phase_voltages(float va, float vb, float vc) {
    if (!_initialised || !_enabled) return ESP_ERR_INVALID_STATE;

    auto set_duty = [&](Phase& phase, float voltage) -> esp_err_t {
        auto bound = fminf(fmaxf(voltage, -1.0f), 1.0f);
        auto normalised = (bound + 1.0) / 2.0;
        uint32_t duty_ticks = (uint32_t)(normalised * (_period_tics - 1));
        return mcpwm_comparator_set_compare_value(phase.cmpr, duty_ticks);
    };

    ESP_RETURN_ON_ERROR(set_duty(phase_a, va), TAG, "phase A failed");
    ESP_RETURN_ON_ERROR(set_duty(phase_b, vb), TAG, "phase B failed");
    ESP_RETURN_ON_ERROR(set_duty(phase_c, vc), TAG, "phase C failed");

    ESP_LOGD(TAG, "Set A=%.3f B=%.3f C=%.3f", va, vb, vc);
    return ESP_OK;
}

esp_err_t Mcpwm::enable() {
    if (!_initialised) return ESP_ERR_INVALID_STATE;
    if (_cfg.n_sleep_gpio != GPIO_NUM_NC) {
        gpio_set_level(_cfg.n_sleep_gpio, 1);
    }
    _enabled = true;
    return ESP_OK;
}

esp_err_t Mcpwm::disable() {
    if (!_initialised) return ESP_ERR_INVALID_STATE;
    ESP_RETURN_ON_ERROR(all_off(), TAG, "disable all_off failed");

    if (_cfg.n_sleep_gpio != GPIO_NUM_NC) {
        gpio_set_level(_cfg.n_sleep_gpio, 0);
    }
    _enabled = false;
    return ESP_OK;
}

esp_err_t Mcpwm::all_off() {
    if (!_initialised) return ESP_ERR_INVALID_STATE;

    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(phase_a.cmpr, 0), TAG, "A off failed");
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(phase_b.cmpr, 0), TAG, "B off failed");
    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(phase_c.cmpr, 0), TAG, "C off failed");

    return ESP_OK;
}