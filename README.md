# RP-26Rune

`RP-26Rune` 是一个可在无相机、无串口硬件条件下直接运行的 RoboMaster
能量机关（大符/小符）全链路开源示例。项目保留插件式自瞄框架的真实组网方式，默认使用随仓库提供的视频和模拟电控状态完成：

```text
模拟串口状态 -> 视频回放 -> OpenVINO 五点检测 -> 传统视觉精修与符运动建模
             -> 火控输出 -> 模拟串口发送
```

## 快速运行

环境要求：Ubuntu 22.04、CMake 3.22+、C++20 编译器、OpenCV、OpenVINO
2024+、Eigen3、Sophus、Ceres、Boost 和 glog。

```bash
./run_demo.sh
```

首次运行会配置并编译工程，随后循环播放
`src/app_plugin/single_camera_manager/config/video/big_rune_demo.mp4`。
启用桌面环境时会显示检测和火控画面。
按 `Ctrl+C` 退出。

在没有桌面环境的 SSH/CI 会话中，可使用 Qt 的离屏平台运行：

```bash
./run_demo.sh --headless
```

`--headless` 不会改写任何配置文件；它只为本次进程设置
`QT_QPA_PLATFORM=offscreen`，算法链路和可视化图像的生成仍会执行，但不会打开桌面窗口。
如果希望完全关闭图像生成和 GUI 线程，请把 `config/runtime_config.json` 中的
`visualize` 改为 `0`。

## 架构与线程组

核心框架位于 `src/core`，业务接入全部位于 `src/app_plugin`：

- `ReceiveDecoder`：按插件自己的 `config/receive_decoder.json` 模拟 ECS/串口输入。
- `CameraManager`：按视频原始 FPS 回放，结束后暂停并循环。
- `NNDetector`：使用开源 RuneDetectionModel 的 OpenVINO 部署方式推理。
- `TrackerManager`：构建与实车一致的 TF 树，并调用 `power_rune::process_power_rune()`。
- `PlannerControl`：调用 `power_rune::get_rune_data()` 并生成 `FireResult`。
- `SendEncoder`：模拟串口发送并输出有限性检查后的结果。

`CameraManager + NNDetector + TrackerManager` 由 `AttachTo` 放入同一个线程组，
保证每个视频帧依次完成检测和符算法处理，不被 LatestBuffer 丢弃；
`PlannerControl + SendEncoder` 同组串行执行。OpenCV HighGUI 只能由
`ImgViz` 内部的唯一 GUI 线程调用，插件禁止直接调用 `imshow()` 或 `waitKey()`。

## 模型部署边界

网络调用只使用开源项目 RuneDetectionModel 的 OpenVINO 流程：BGR/NHWC u8
输入、RGB/f32 归一化、letterbox、五关键点解码和中心距离 NMS。关键点顺序固定为：

```text
0=top, 1=left, 2=point_R, 3=right, 4=bottom
```

传统视觉精修、平面重建、相位运动估计和决策均由 `src/core/algorithm/power_rune`
完成。项目不链接或分发非公开参考工程中的网络部署库及私有模型。

随仓演示参数为 `confidence_threshold=0.65`、
`keypoint_confidence_threshold=0.5`、`min_valid_keypoints=5`；这些参数针对演示素材
设置，与上游示例默认的 `0.8`、`0.8`、`3` 不同。阈值变化不改变 OpenVINO
预处理、输出解码和 NMS 的实现方式。

模型版权信息见
[detector 第三方声明](src/app_plugin/detector/THIRD_PARTY_NOTICE.md)。

## 配置

- `config/runtime_config.json`：插件加载顺序、线程组和 GUI 总开关。
- `config/power_rune.json`：传统前处理、平面重建、运动模型及火控参数。
- `src/app_plugin/receive_decoder/config/receive_decoder.json`：模拟电控数据。
- `src/app_plugin/single_camera_manager/config/camera.json`：视频回放参数。
- `src/app_plugin/detector/config/detect.json`：模型路径、推理设备和检测阈值。
- `src/app_plugin/tracker_manager/config/track.json`：相机相对云台的 TF 参数。
- `src/app_plugin/fire_control_system/config/fire_control_system.json`：火控显示参数。

`runtime_config.json` 中的 `visualize` 是 `ImgViz` 的进程级总开关；关闭后不会启动
HighGUI 线程。`fire_control_system.json` 中的 `visualize.enabled` 只控制
`PlannerControl` 是否提交最终火控叠加画面，不能关闭检测器提交的调试画面。要关闭
全部窗口及图像提交，使用前者。

`receive_decoder.json` 的 `mode` 使用框架枚举：`2=SmallRune`、`3=BigRune`；
`my_color` 使用 `0=Red`、`1=Blue`。更换演示视频时，应同步确认
模式、目标颜色、1440×1080 分辨率以及 `power_rune.json` 中的相机标定参数。

`detect.json` 的 `rune_detect.device` 默认使用 `CPU`，确保没有 Intel GPU 的机器也能
运行；可改为 `AUTO` 或 `GPU`。ONNX 网络本体位于同一插件的
`config/openvino/model-0624.onnx`。演示视频必须保持 1440×1080，以匹配默认相机内参。

随仓视频使用 H.264 编码，因此 OpenCV `videoio` 还需要可用的 FFmpeg 或 GStreamer
H.264 解码后端；缺少该能力时 `CameraManager` 会在打开视频时报错。

## 自定义插件

插件通过 `REGISTER_PLUGIN(...)` 注册为动态库，运行时不直接链接具体插件。
通信应在 `declare()` 中通过 `Context` 明确声明，并在 `process()` 中缓存端点、处理一轮后返回。
详细规范见：

- [core 使用说明](src/core/README.md)
- [Context 通信组网指南](src/core/app/context/README.md)
- [Power Rune 外部接口](src/core/algorithm/power_rune/docs/README.md)

## 许可证

项目自有代码采用 [MIT License](LICENSE)。第三方模型及部署实现遵循其对应许可证。
