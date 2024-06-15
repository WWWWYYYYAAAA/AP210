#ifndef _FATFS_H_
#define _FATFS_H_

#include <stdio.h>
#include <errno.h>
#include <dirent.h>
#include "esp_console.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "tinyusb.h"
#include "tusb_msc_storage.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef CONFIG_EXAMPLE_STORAGE_MEDIA_SDMMCCARD
#include "diskio_impl.h"
#include "diskio_sdmmc.h"

#endif

static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;

void init_myfatfs();
int8_t IsExist(char *filename);
int8_t FAT_regular_init();
void FAT_unmount();
int8_t FAT_format_init();
#endif