#include "cust_baro.h"

struct baro_hw cust_baro_hw[] __attribute__((section(".cust_baro"))) = {
#ifdef CFG_BMP280_SUPPORT
    {
        .name = "bmp280",
        .i2c_num = 1,
        .direction = 0,
        .i2c_addr = {0x77, 0},
    },
#endif
#ifdef CFG_QMP6988_SUPPORT
    {
        .name = "qmp6988",
        .i2c_num = 1,
        .direction = 0,
        .i2c_addr = {0x70, 0x56},
    },
#endif
};
