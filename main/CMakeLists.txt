set(requires fatfs console)

if(CONFIG_EXAMPLE_STORAGE_MEDIA_SPIFLASH)
    list(APPEND requires wear_levelling)
endif()

idf_component_register(#SRCS "fatfs.c" "imu.c" "i2cRW.c" "main.c"
                    SRC_DIRS "./"
                    INCLUDE_DIRS "../include/"
                    REQUIRES "${requires}")
