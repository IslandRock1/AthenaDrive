#include <stdio.h>
#include "SDCard.hpp"

SDCard::SDCard() {}

esp_err_t SDCard::writeFile(const char *filename, char *format, ...) {
    char path[128];
    snprintf(path, sizeof(path), "/sdcard/%s.txt", filename);

    ESP_LOGI("SDCard", "Opening file %s", path);
    FILE *f = fopen(path, "a");
    if (f == NULL) {
        ESP_LOGE("SDCard", "Failed to open file for writing");
        return ESP_FAIL;
    }

    // Need a bigger buffer if
    // the print is too long.
    char buffer[256];

    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    fprintf(f, "%s", buffer);

    fclose(f);
    ESP_LOGI("SDCard", "File written");

    return ESP_OK;
}