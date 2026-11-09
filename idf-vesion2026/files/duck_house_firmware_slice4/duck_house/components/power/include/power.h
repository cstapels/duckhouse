#pragma once
#include "esp_err.h"

/* Light-sleep manager (software spec section 7, Sleep and Wake). */
esp_err_t power_init(void);
