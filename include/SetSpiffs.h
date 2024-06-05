#ifndef _SETSPIFFS_H_
#define _SETSPIFFS_H_

#include <stdio.h>
#include "esp_spiffs.h"
#include "esp_log.h"

int8_t user_partition_init();
void delete_SPIFFS();
#endif