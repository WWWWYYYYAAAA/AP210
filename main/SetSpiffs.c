#include "SetSpiffs.h"

int8_t user_partition_init(){
     // 初始化 SPIFFS
    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/user_partition",  // 指定 SPIFFS 的挂载路径
      .partition_label = "SPIFFS",  // 分区标签，如果为 NULL，则使用默认的 SPIFFS 分区
      .max_files = 5,  // SPIFFS 可以打开的最大文件数
      .format_if_mount_failed = true  // 如果挂载失败，是否格式化 SPIFFS
    };

    // 注册 SPIFFS 到 VFS
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    // 检查 SPIFFS 是否成功初始化
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE("TAG", "无法挂载或格式化文件系统");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE("TAG", "未找到 SPIFFS 分区");
        } else {
            ESP_LOGE("TAG", "无法初始化 SPIFFS (%s)", esp_err_to_name(ret));
        }
        return -1;
    }
    return 0;
}

void delete_SPIFFS()
{
    esp_vfs_spiffs_unregister(NULL);  // 从 VFS 中注销 SPIFFS
}