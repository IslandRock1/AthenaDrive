
#pragma once

#include "driver/gpio.h"

#include "stdarg.h"
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

class SDCard {
public:
    SDCard();

    static esp_err_t writeFile(const char *filename, char *format, ...);
};