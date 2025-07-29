#ifndef BSP_IIS_MAX98357A_H
#define BSP_IIS_MAX98357A_H

#include "driver/i2s_std.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* I2S GPIO Configuration */
#define BSP_I2S_BCLK_PIN    (GPIO_NUM_39)
#define BSP_I2S_WS_PIN      (GPIO_NUM_37)
#define BSP_I2S_DOUT_PIN    (GPIO_NUM_11)

/**
 * @brief Initialize I2S for MAX98357A
 *
 * @param[in] sample_rate Audio sample rate (e.g., 44100, 16000)
 * @return 
 *      - ESP_OK on success
 *      - ESP_FAIL on error
 */
esp_err_t bsp_iis_max98357a_init(uint32_t sample_rate);

/**
 * @brief Write audio data to I2S
 *
 * @param[in] src Pointer to the data buffer
 * @param[in] data_size Size of the data in bytes
 * @param[out] bytes_written Number of bytes actually written
 * @param[in] timeout_ms Timeout in milliseconds
 *
 * @return 
 *      - ESP_OK on success
 *      - ESP_FAIL on error
 */
esp_err_t bsp_iis_max98357a_write(const void *src, size_t data_size, size_t *bytes_written, uint32_t timeout_ms);

/**
 * @brief Deinitialize I2S
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL on error
 */
esp_err_t bsp_iis_max98357a_deinit(void);


#ifdef __cplusplus
}
#endif

#endif /* BSP_IIS_MAX98357A_H */

