#include "SetSpiffs.h"

int8_t user_partition_init(){
     // 初始化 SPIFFS
    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/user_partition",
      .partition_label = "SPIFFS", 
      .max_files = 8, 
      .format_if_mount_failed = false
    };

    // 注册 SPIFFS 到 VFS
    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));
    // esp_err_t ret = esp_vfs_spiffs_register(&conf);
    // //printf("ret %d\n", ret);
    // // 检查 SPIFFS 是否成功初始化
    // if (ret != ESP_OK) {
    //     if (ret == ESP_FAIL) {
    //         ESP_LOGE("TAG", "mount or formatfailed");
    //     } else if (ret == ESP_ERR_NOT_FOUND) {
    //         ESP_LOGE("TAG", "no SPIFFS partition");
    //     } else {
    //         ESP_LOGE("TAG", "initialize SPIFFS failed (%s)", esp_err_to_name(ret));
    //     }
    //     return -1;
    // }
    return 0;
}

void delete_SPIFFS()
{
    esp_vfs_spiffs_unregister(NULL);  // 从 VFS 中注销 SPIFFS
}

int8_t IsExist(char *filename)
{
    FILE * f0 = fopen(filename, "r"); 
    if (f0!=NULL)
    {
        fclose(f0);
        return 1;
    }
    else
    {
        return 0;
    }
}