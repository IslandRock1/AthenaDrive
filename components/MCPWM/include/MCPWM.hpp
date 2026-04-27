#pragma once

#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include <cmath>
#include <cstdint>

class Mcpwm {
public:
    struct Config {
        gpio_num_t pwm_a_gpio;
        gpio_num_t pwm_b_gpio;
        gpio_num_t pwm_c_gpio;
        gpio_num_t n_sleep_gpio;

        uint32_t pwm_freq_hz = 30000;
        int group_id = 0;
        uint32_t timer_resolution_hz = 100000000; // 100 MHz
    };

    esp_err_t init(const Config& cfg);
    esp_err_t set_phase_voltages(float va, float vb, float vc); // [-1, 1]
    bool is_initialised() const { return _initialised; }

    esp_err_t enable();
    esp_err_t disable();

private:
    struct Phase {
        mcpwm_oper_handle_t oper = nullptr;
        mcpwm_cmpr_handle_t cmpr = nullptr;
        mcpwm_gen_handle_t gen = nullptr;
    };

    esp_err_t init_single_phase(Phase& phase, gpio_num_t pwm_gpio);
    uint32_t safe_compare_from_normalised(float normalised) const;

    Config _cfg{};
    uint32_t _period_tics = 0;
    bool _initialised = false;
    bool _enabled = false;

    mcpwm_timer_handle_t _timer = nullptr;
    Phase phase_a{}, phase_b{}, phase_c{};
};