#ifndef ZEPHYR_DRIVERS_SENSOR_MPR121_MPR121_H_
#define ZEPHYR_DRIVERS_SENSOR_MPR121_MPR121_H_

#include <zephyr/device.h>
#include <stdint.h>
#include <stdbool.h>

#define MPR121_TOUCH_STATUS_0       0x00
#define MPR121_TOUCH_STATUS_1       0x01
#define MPR121_ELE_DATA_BASE        0x04
#define MPR121_ELEPROX_DATA_LSB     0x1C
#define MPR121_ELEPROX_DATA_MSB     0x1D
#define MPR121_BASELINE_BASE        0x1E
#define MPR121_ELEPROX_BASELINE     0x2A

#define MPR121_MHD_RISING           0x2B
#define MPR121_NHD_RISING           0x2C
#define MPR121_NCL_RISING           0x2D
#define MPR121_FDL_RISING           0x2E
#define MPR121_MHD_FALLING          0x2F
#define MPR121_NHD_FALLING          0x30
#define MPR121_NCL_FALLING          0x31
#define MPR121_FDL_FALLING          0x32
#define MPR121_NHD_TOUCHED          0x33
#define MPR121_NCL_TOUCHED          0x34
#define MPR121_FDL_TOUCHED          0x35

#define MPR121_PROX_MHD_RISING      0x36
#define MPR121_PROX_NHD_RISING      0x37
#define MPR121_PROX_NCL_RISING      0x38
#define MPR121_PROX_FDL_RISING      0x39
#define MPR121_PROX_MHD_FALLING     0x3A
#define MPR121_PROX_NHD_FALLING     0x3B
#define MPR121_PROX_NCL_FALLING     0x3C
#define MPR121_PROX_FDL_FALLING     0x3D
#define MPR121_PROX_NHD_TOUCHED     0x3E
#define MPR121_PROX_NCL_TOUCHED     0x3F
#define MPR121_PROX_FDL_TOUCHED     0x40

#define MPR121_ELE_TOUCH_TH_BASE    0x41
#define MPR121_ELE_RELEASE_TH_BASE  0x42
#define MPR121_PROX_TOUCH_TH        0x59
#define MPR121_PROX_RELEASE_TH      0x5A
#define MPR121_DEBOUNCE             0x5B
#define MPR121_AFE_CONFIG           0x5C
#define MPR121_FILTER_CONFIG        0x5D
#define MPR121_ECR                  0x5E

#define MPR121_AUTO_CONFIG_CTRL0    0x7B
#define MPR121_AUTO_CONFIG_CTRL1    0x7C
#define MPR121_AUTO_CONFIG_USL      0x7D
#define MPR121_AUTO_CONFIG_LSL      0x7E
#define MPR121_AUTO_CONFIG_TL       0x7F
#define MPR121_SOFT_RESET           0x80
#define MPR121_SOFT_RESET_CMD       0x63

#define MPR121_ESI_1MS              0x00
#define MPR121_ESI_128MS            0x07
#define MPR121_ELEPROX_STATUS_BIT   BIT(4)
#define MPR121_MAX_ELECTRODES       12

struct mpr121_sample {
    uint16_t filtered[MPR121_MAX_ELECTRODES];
    uint16_t baseline[MPR121_MAX_ELECTRODES];
    uint16_t touch_status;
    bool     proximity_active;
    uint16_t prox_filtered;
    uint16_t prox_baseline;
    uint8_t  electrode_count;
};

typedef void (*mpr121_data_ready_cb_t)(const struct device *dev,
                                       struct mpr121_sample *sample,
                                       void *user_data);

int mpr121_read_sample(const struct device *dev, struct mpr121_sample *sample);
int mpr121_set_esi(const struct device *dev, uint8_t esi);
int mpr121_register_data_ready_cb(const struct device *dev,
                                  mpr121_data_ready_cb_t cb, void *user_data);

#endif /* ZEPHYR_DRIVERS_SENSOR_MPR121_MPR121_H_ */
