#include "Watchdog.h"

#include <Arduino.h>

#include "Config.h"

#if __has_include("esp_task_wdt.h")
#include "esp_task_wdt.h"
#define W209_HAS_TASK_WDT 1
#else
#define W209_HAS_TASK_WDT 0
#endif

#if __has_include("esp_idf_version.h")
#include "esp_idf_version.h"
#endif

namespace Watchdog {

static bool enabled = false;

void begin() {
    if (!Config::ENABLE_TASK_WATCHDOG) {
        enabled = false;
        return;
    }

#if W209_HAS_TASK_WDT
    esp_err_t result = ESP_OK;

#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 5
    esp_task_wdt_config_t config = {};
    config.timeout_ms = Config::WATCHDOG_TIMEOUT_MS;
    config.idle_core_mask = 0;
    config.trigger_panic = true;

    result = esp_task_wdt_init(&config);

    if (result == ESP_ERR_INVALID_STATE) {
        result = esp_task_wdt_reconfigure(&config);
    }
#else
    result = esp_task_wdt_init(
        Config::WATCHDOG_TIMEOUT_MS / 1000,
        true
    );
#endif

    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
        enabled = false;
        return;
    }

    result = esp_task_wdt_add(nullptr);

    if (result == ESP_OK || result == ESP_ERR_INVALID_STATE) {
        enabled = true;
    }
#else
    enabled = false;
#endif
}

void feed() {
#if W209_HAS_TASK_WDT
    if (enabled) {
        esp_task_wdt_reset();
    }
#endif
}

} // namespace Watchdog
