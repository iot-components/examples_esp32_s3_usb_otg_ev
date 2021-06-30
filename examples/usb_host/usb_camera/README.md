## USB Camera LCD Display Demo 说明

该示例程序代码仅用于 ESP32-S2 USB Host UVC 功能测试与评估，不建议基于此开发量产产品，原因请查看 [/README.md](../../README.md)

* 支持 USB Camera 数据流获取和解析
* 支持 JPEG 本地软件解码
* 支持 SPI LCD 屏幕显示

### 硬件准备

* ESP32-S2 USB 摄像头选型需要满足的参数（2021.03）：

    1. 摄像头兼容 USB1.1 全速模式
    2. 摄像头自带 MJPEG 压缩
    3. 摄像头支持设置端点`wMaxPacketSize`为`512`
    4. MJPEG 支持 **320*240** 分辨率
    5. MJPEG 支持设置帧率到 15 帧/s

* ESP32-S2 USB 摄像头硬件接线：
  1. USB 摄像头 VBUS 请使用 5V 电源独立供电，亦可使用 IO 控制 VBUS 通断
  2. UBS 摄像头 D+ D- 数据线请按常规差分信号标准走线
  3. USB 摄像头 D+ (绿线) 接 ESP32-S2 GPIO20
  4. USB 摄像头 D- (白线) 接 ESP32-S2 GPIO19

### 编译示例代码

示例代码基于 `esp32-s2-kaluga` 开发板编写，可按以下过程直接编译烧写：

1. 确认 ESP-IDF 环境成功搭建，并按照说明文件添加补丁 [usb_support_workaround.patch](../../idf_patch/readme.md)
2. 添加 ESP-IDF 环境变量，Linux 方法如下，其它平台请查阅 [Set up the environment variables](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html#step-4-set-up-the-environment-variables)
    ```
    . $HOME/esp/esp-idf/export.sh
    ```
3. 设置编译目标为 `esp32-s2`
    ```
    idf.py set-target esp32s2
    ```
4. 编译、下载、查看输出
    ```
    idf.py build flash monitor
    ```

## 使用说明

1. 注意 LCD 屏幕和摄像头接口接线即可
2. 屏幕默认直接输出摄像头图像
3. 用户可通过使能 `boot animation` 开启开机动画，验证 LCD 屏幕是否能够成功点亮

## 性能参数

**ESP32-S2 240Mhz**：

| 典型分辨率  | USB 典型传输帧率 | JPEG 最大解码+显示帧率* |
| :-----: | :--------------: | :----------------------: |
| 320*240 |        15        |           ~12            |
| 160*120 |        30        |           ~28            |
|         |                  |                          |

1. *JPEG 解码+ 显示帧率随 CPU 负载波动
2. 其它分辨率以实际测试为准，或按照解码时间和分辨率正相关估算

### 二次开发

API 使用说明请参考：[common_components/uvc_stream/README](../../common_components/uvc_stream/README.md)
