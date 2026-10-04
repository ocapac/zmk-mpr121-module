#define DT_DRV_COMPAT freescale_mpr121

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "mpr121.h"

LOG_MODULE_REGISTER(mpr121, CONFIG_MPR121_LOG_LEVEL);

struct mpr121_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec int_gpio;
    uint8_t touch_threshold;
    uint8_t release_threshold;
    uint8_t prox_touch_threshold;
    uint8_t prox_release_threshold;
    uint8_t electrode_count;
    bool    enable_proximity;
    uint8_t debounce;
    uint8_t afe_config;
    uint8_t filter_config;
    uint8_t auto_config_usl;
    uint8_t auto_config_lsl;
    uint8_t auto_config_tl;
};

struct mpr121_data {
    const struct device *dev;
    struct gpio_callback gpio_cb;
    struct k_work work;
    mpr121_data_ready_cb_t data_ready_cb;
    void *data_ready_user_data;
};

static int mpr121_write_reg(const struct device *dev, uint8_t reg, uint8_t val)
{
    const struct mpr121_config *cfg = dev->config;
    uint8_t buf[2] = { reg, val };
    return i2c_write_dt(&cfg->i2c, buf, sizeof(buf));
}

static int mpr121_read_regs(const struct device *dev, uint8_t reg,
                            uint8_t *buf, size_t len)
{
    const struct mpr121_config *cfg = dev->config;
    return i2c_write_read_dt(&cfg->i2c, &reg, 1, buf, len);
}

int mpr121_set_esi(const struct device *dev, uint8_t esi)
{
    uint8_t val;
    int ret;

    if (esi > MPR121_ESI_128MS) return -EINVAL;

    ret = mpr121_read_regs(dev, MPR121_FILTER_CONFIG, &val, 1);
    if (ret < 0) return ret;

    val &= ~0x07;
    val |= (esi & 0x07);
    return mpr121_write_reg(dev, MPR121_FILTER_CONFIG, val);
}

int mpr121_read_sample(const struct device *dev, struct mpr121_sample *sample)
{
    const struct mpr121_config *cfg = dev->config;
    uint8_t buf[0x2B]; /* 0x00..0x2A */
    int ret;

    ret = mpr121_read_regs(dev, 0x00, buf, sizeof(buf));
    if (ret < 0) return ret;

    sample->touch_status = buf[0x00] | ((buf[0x01] & 0x0F) << 8);
    sample->proximity_active = (buf[0x01] & MPR121_ELEPROX_STATUS_BIT) != 0;

    for (int i = 0; i < cfg->electrode_count; i++) {
        uint8_t lsb = buf[0x04 + i * 2];
        uint8_t msb = buf[0x05 + i * 2];
        sample->filtered[i] = (uint16_t)lsb | ((uint16_t)(msb & 0x03) << 8);
        sample->baseline[i] = (uint16_t)buf[0x1E + i] << 2;
    }

    sample->prox_filtered = (uint16_t)buf[0x1C] |
                            ((uint16_t)(buf[0x1D] & 0x03) << 8);
    sample->prox_baseline = (uint16_t)buf[0x2A] << 2;
    sample->electrode_count = cfg->electrode_count;
    return 0;
}

int mpr121_register_data_ready_cb(const struct device *dev,
                                  mpr121_data_ready_cb_t cb, void *user_data)
{
    struct mpr121_data *data = dev->data;
    data->data_ready_cb = cb;
    data->data_ready_user_data = user_data;
    return 0;
}

static void mpr121_work_handler(struct k_work *work)
{
    struct mpr121_data *data = CONTAINER_OF(work, struct mpr121_data, work);
    const struct device *dev = data->dev;
    struct mpr121_sample sample;

    if (mpr121_read_sample(dev, &sample) < 0) return;
    if (data->data_ready_cb) {
        data->data_ready_cb(dev, &sample, data->data_ready_user_data);
    }
}

static void mpr121_gpio_callback(const struct device *dev,
                                 struct gpio_callback *cb, uint32_t pins)
{
    struct mpr121_data *data = CONTAINER_OF(cb, struct mpr121_data, gpio_cb);
    k_work_submit(&data->work);
}

static int mpr121_init(const struct device *dev)
{
    const struct mpr121_config *cfg = dev->config;
    struct mpr121_data *data = dev->data;
    int ret;

    data->dev = dev;
    k_work_init(&data->work, mpr121_work_handler);

    if (!i2c_is_ready_dt(&cfg->i2c)) return -ENODEV;
    if (!gpio_is_ready_dt(&cfg->int_gpio)) return -ENODEV;

    gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&data->gpio_cb, mpr121_gpio_callback,
                       BIT(cfg->int_gpio.pin));
    gpio_add_callback(cfg->int_gpio.port, &data->gpio_cb);

    /* Soft reset */
    mpr121_write_reg(dev, MPR121_SOFT_RESET, MPR121_SOFT_RESET_CMD);
    k_msleep(10);

    /* Baseline filters - electrodes */
    mpr121_write_reg(dev, MPR121_MHD_RISING,   0x01);
    mpr121_write_reg(dev, MPR121_NHD_RISING,   0x01);
    mpr121_write_reg(dev, MPR121_NCL_RISING,   0x00);
    mpr121_write_reg(dev, MPR121_FDL_RISING,   0x00);
    mpr121_write_reg(dev, MPR121_MHD_FALLING,  0x01);
    mpr121_write_reg(dev, MPR121_NHD_FALLING,  0x01);
    mpr121_write_reg(dev, MPR121_NCL_FALLING,  0xFF);
    mpr121_write_reg(dev, MPR121_FDL_FALLING,  0x02);
    mpr121_write_reg(dev, MPR121_NHD_TOUCHED,  0x00);
    mpr121_write_reg(dev, MPR121_NCL_TOUCHED,  0x00);
    mpr121_write_reg(dev, MPR121_FDL_TOUCHED,  0x00);

    /* Baseline filters - ELEPROX (AN3893 Table 8) */
    mpr121_write_reg(dev, MPR121_PROX_MHD_RISING,   0xFF);
    mpr121_write_reg(dev, MPR121_PROX_NHD_RISING,   0xFF);
    mpr121_write_reg(dev, MPR121_PROX_NCL_RISING,   0x00);
    mpr121_write_reg(dev, MPR121_PROX_FDL_RISING,   0x00);
    mpr121_write_reg(dev, MPR121_PROX_MHD_FALLING,  0x01);
    mpr121_write_reg(dev, MPR121_PROX_NHD_FALLING,  0x01);
    mpr121_write_reg(dev, MPR121_PROX_NCL_FALLING,  0xFF);
    mpr121_write_reg(dev, MPR121_PROX_FDL_FALLING,  0xFF);
    mpr121_write_reg(dev, MPR121_PROX_NHD_TOUCHED,  0x00);
    mpr121_write_reg(dev, MPR121_PROX_NCL_TOUCHED,  0x00);
    mpr121_write_reg(dev, MPR121_PROX_FDL_TOUCHED,  0x00);

    /* Touch/release thresholds */
    for (int i = 0; i < cfg->electrode_count; i++) {
        mpr121_write_reg(dev, MPR121_ELE_TOUCH_TH_BASE + i * 2,
                         cfg->touch_threshold);
        mpr121_write_reg(dev, MPR121_ELE_RELEASE_TH_BASE + i * 2,
                         cfg->release_threshold);
    }
    mpr121_write_reg(dev, MPR121_PROX_TOUCH_TH, cfg->prox_touch_threshold);
    mpr121_write_reg(dev, MPR121_PROX_RELEASE_TH, cfg->prox_release_threshold);

    /* Debounce */
    mpr121_write_reg(dev, MPR121_DEBOUNCE, cfg->debounce);

    /* AFE + Filter */
    mpr121_write_reg(dev, MPR121_AFE_CONFIG, cfg->afe_config);
    mpr121_write_reg(dev, MPR121_FILTER_CONFIG, cfg->filter_config);

    /* Auto-config */
    mpr121_write_reg(dev, MPR121_AUTO_CONFIG_CTRL0, 0x0B);
    mpr121_write_reg(dev, MPR121_AUTO_CONFIG_CTRL1, 0x00);
    mpr121_write_reg(dev, MPR121_AUTO_CONFIG_USL, cfg->auto_config_usl);
    mpr121_write_reg(dev, MPR121_AUTO_CONFIG_LSL, cfg->auto_config_lsl);
    mpr121_write_reg(dev, MPR121_AUTO_CONFIG_TL,  cfg->auto_config_tl);

    /* ECR: CL=10, ELEPROX_EN=11, ELE_EN=8 electrodes */
    uint8_t ecr = 0x80;
    if (cfg->enable_proximity) ecr |= 0x30;
    ecr |= (cfg->electrode_count & 0x0F);
    mpr121_write_reg(dev, MPR121_ECR, ecr);

    LOG_INF("MPR121 initialised: %d electrodes, prox=%s",
            cfg->electrode_count, cfg->enable_proximity ? "on" : "off");
    return 0;
}

#define MPR121_INIT(inst)                                                   \
    static struct mpr121_config mpr121_config_##inst = {                    \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                 \
        .int_gpio = GPIO_DT_SPEC_INST_GET(inst, int_gpios),                \
        .touch_threshold = DT_INST_PROP(inst, touch_threshold),            \
        .release_threshold = DT_INST_PROP(inst, release_threshold),        \
        .prox_touch_threshold = DT_INST_PROP(inst, prox_touch_threshold),  \
        .prox_release_threshold = DT_INST_PROP(inst, prox_release_threshold), \
        .electrode_count = DT_INST_PROP(inst, electrode_count),            \
        .enable_proximity = DT_INST_PROP(inst, enable_proximity),          \
        .debounce = DT_INST_PROP(inst, debounce),                          \
        .afe_config = DT_INST_PROP(inst, afe_configuration),               \
        .filter_config = DT_INST_PROP(inst, filter_configuration),         \
        .auto_config_usl = DT_INST_PROP(inst, auto_config_usl),            \
        .auto_config_lsl = DT_INST_PROP(inst, auto_config_lsl),            \
        .auto_config_tl = DT_INST_PROP(inst, auto_config_tl),              \
    };                                                                      \
    static struct mpr121_data mpr121_data_##inst;                           \
    DEVICE_DT_INST_DEFINE(inst, mpr121_init, NULL,                          \
                          &mpr121_data_##inst, &mpr121_config_##inst,       \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(MPR121_INIT)
