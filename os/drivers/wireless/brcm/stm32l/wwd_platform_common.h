
/**
 * WLAN SDIO pins
 */
typedef enum
{
    WWD_PIN_SDIO_OOB_IRQ,
    WWD_PIN_SDIO_CLK,
    WWD_PIN_SDIO_CMD,
    WWD_PIN_SDIO_D0,
    WWD_PIN_SDIO_D1,
    WWD_PIN_SDIO_D2,
    WWD_PIN_SDIO_D3,
    WWD_PIN_SDIO_MAX,
} wwd_sdio_pin_t;

extern const platform_gpio_t wifi_sdio_pins   [];
