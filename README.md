---

# ESP32 LVGL + Lottie + 多传感器集成项目

## 项目简介

本项目基于 ESP32 平台，集成了 LVGL 图形库、Lottie 动画显示，并支持多种传感器（DS18B20 温度、LTR390UV 紫外、AS7341 光谱、ADC 多通道模拟采集）。项目采用分层 BSP 驱动架构，驱动层专注采集，应用层负责业务逻辑，适合二次开发和功能扩展。

## 主要功能

- LVGL 图形界面与 Lottie 动画流畅集成，内存与任务调度优化，支持动画与传感器数据同屏显示。
- 支持 DS18B20、LTR390UV、AS7341、ADC（多通道）等多种传感器，采集任务与主业务分离，BSP 化驱动。
- ADC 支持校准（ESP-IDF v5.x 推荐 API），自动降级为线性换算，采集值可直接换算为电压和百分比。
- 代码结构清晰，易于维护和扩展。

## 目录结构

```
main/
  ├── main.c                // 主应用入口，集成 LVGL、Lottie、各传感器采集与显示
  ├── CMakeLists.txt        // 组件构建配置
  ├── idf_component.yml     // 组件依赖声明
  ├── bsp/                  // BSP 驱动层
  │   ├── include/          // 各传感器/功能头文件
  │   └── src/              // 各传感器/功能实现
  └── lottie_img/           // Lottie 动画资源
      └── json/             // Lottie 动画 JSON 文件
components/
  └── esp_lvgl_port/        // LVGL 移植与相关示例
```

## 依赖环境

- ESP-IDF >= 4.1.0（推荐 5.x 及以上）
- 主要依赖组件（见 `idf_component.yml` 和 `CMakeLists.txt`）：
  - LVGL
  - esp_lvgl_port
  - esp_rlottie
  - teriyakigod/esp_lcd_st7735
  - k0i05/esp_ltr390uv
  - k0i05/esp_as7341
  - espressif/ds18b20
  - espressif/onewire_bus
  - esp_adc

## 编译与烧录

1. 安装 ESP-IDF 并配置好环境变量。
2. 拉取所有子模块和依赖组件：
   ```sh
   idf.py add-dependency
   ```
3. 配置硬件参数（如 LCD、ADC、I2C、GPIO 等），可通过 `menuconfig` 或直接修改 `sdkconfig`。
4. 编译并烧录：
   ```sh
   idf.py build
   idf.py -p (你的串口) flash
   ```

## 主要 BSP API 说明

### ADC 多通道采集

- 初始化与启动
  ```c
  bsp_adc_config_t adc_cfg = {
      .channels = { {ADC1_CHANNEL_0, 1} },
      .channel_num = 1,
      .sample_freq_hz = 10,
  };
  bsp_adc_init(&adc_cfg);
  bsp_adc_start();
  ```
- 获取最新采样值
  ```c
  uint16_t values[BSP_ADC_MAX_CHANNELS];
  int ch_num = 0;
  bsp_adc_get_latest(values, &ch_num);
  ```

### DS18B20 温度传感器

- 初始化与任务启动
  ```c
  bsp_ds18b20_init();
  bsp_ds18b20_start_read_task(3, 4096);
  ```

### LTR390UV 紫外传感器

- 初始化与任务启动
  ```c
  bsp_ltr390uv_init();
  bsp_ltr390uv_start_read_task(4, 4096);
  ```

### AS7341 光谱传感器

- 初始化
  ```c
  bsp_as7341_init();
  // 采集接口可根据实际需求扩展
  ```

## Lottie 动画集成

- 动画资源存放于 `main/lottie_img/json/`，可通过 `lv_rlottie_create_from_raw` 加载显示。
- 内存不足时自动降级为简单文本提示，保证系统稳定。

## 常见问题与建议

- **WDT 超时/栈溢出**：建议简化动画、加大任务栈、优化任务优先级。
- **内存不足**：可通过 `esp_get_free_heap_size()` 动态判断，必要时降级显示。
- **BSP 层与应用层分工**：驱动层只做采集，应用层做业务处理，便于维护。
- **ADC 校准失败**：自动降级为线性换算，保证采集功能健壮。

## 参考示例

主应用 `main.c` 已集成 LVGL+Lottie+多传感器完整示例，详细见代码注释。

---

如需扩展更多传感器或功能，建议参考 BSP 驱动模板，保持分层结构和统一风格。

如有问题欢迎提 issue 或交流！

