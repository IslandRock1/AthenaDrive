
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "stdatomic.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class OneshotADC {
public:
    OneshotADC();

    void startTasks();
    static void taskAEntry(void *arg);
    static void taskBEntry(void *arg);
    static void taskCEntry(void *arg);

    void taskA();
    void taskB();
    void taskC();

    int getA();
    int getB();
    int getC();

private:
    void readA();
    void readB();
    void readC();

    adc_oneshot_unit_handle_t adc1_handle;

    bool do_calibration1_chan0 = false;
    bool do_calibration1_chan1 = false;
    bool do_calibration1_chan2 = false;
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    adc_cali_handle_t adc1_cali_chan1_handle = NULL;
    adc_cali_handle_t adc1_cali_chan2_handle = NULL;

    int rawA, rawB, rawC;
    int calcA, calcB, calcC;

    atomic_int voltageA = 0;
    atomic_int voltageB = 0;
    atomic_int voltageC = 0;
};