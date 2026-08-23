# Power Rune 外部调用说明

外部模块只需要包含 `power_rune_interface.hpp` 并链接
`power_rune_interface_lib`，不需要直接依赖 `PowerRuneProcessor`、
`RuneDecisionModule` 等内部实现。

## CMake 配置

确保工程已经通过 `add_subdirectory` 加载 `power_rune` 目录，然后为调用目标添加：

```cmake
target_link_libraries(your_target PRIVATE power_rune_interface_lib)
```

`power_rune_interface_lib` 会传递接口头文件、所需依赖以及实际算法库
`power_rune_lib`。

## 配置说明

[power_rune.jsonc](power_rune.jsonc) 是打符配置的带注释示例，说明了相机标定、检测、投影、相位估计、火控、弹道和调试开关各配置项的含义与单位。

运行时读取的是部署目录中的 `config/power_rune.json`。使用时请将示例中的键和值复制到该文件；`power_rune.jsonc` 含有 `//` 注释，仅用于阅读和调参说明，不能直接作为运行配置加载。

## 调用方式

两个接口可以分离调用，不要求在同一线程中执行，也不要求每次
`process_power_rune()` 后立即调用 `get_rune_data()`：

- 图像处理线程构造 `power_rune::RuneInput`，调用
  `power_rune::process_power_rune()` 更新符算法。
- 火控发送线程按自身频率调用 `power_rune::get_rune_data()`，获取当前输出。

### 图像处理线程

```cpp
#include "power_rune_interface.hpp"

power_rune::RuneInput input = /* 构造本帧符输入 */;
power_rune::process_power_rune(input);
```

构造好 `RuneInput` 后直接调用 `process_power_rune()` 即可。

### 火控发送线程

```cpp
#include "power_rune_interface.hpp"

power_rune::RuneSendData read_power_rune_result(bool is_big_rune)
{
    return power_rune::get_rune_data(is_big_rune);
}
```

`get_rune_data()` 返回符算法当前保存的输出。它可以按照火控发送线程自己的频率调用，
无需与图像处理的频率对应。

## 输入字段

| 字段 | 说明 |
| --- | --- |
| `is_big_rune` | `true` 表示大符，`false` 表示小符 |
| `ori_mat` | 当前帧原图 |
| `tf_tree` | 当前帧使用的坐标变换树 |
| `timestamp` | 原图采集时间 |
| `cd_my_color` | 电控发来的my_color |
| `nn_rune_infos` | 神经网络检测到的符叶信息 |
| `class_id` | `0` 表示未击打，`1` 表示已击打 |

每个检测结果需要提供 `top`、`left`、`right`、`bottom` 和符心
`point_R` 五个关键点。

## 输出字段

`power_rune::get_rune_data()` 返回 `power_rune::RuneSendData`：

| 字段 | 说明 |
| --- | --- |
| `yaw` | 目标偏航角，单位为弧度（rad） |
| `pitch` | 目标俯仰角，单位为弧度（rad） |
| `is_find_buff` | 是否找到有效符目标 |
| `mode` | 火控模式 |
| `is_enable_fire` | 是否允许开火 |

## 线程约束

这两个接口不是多调用线程安全的，必须遵守以下约束：

- `process_power_rune()` 只能由一个固定的图像处理线程调用，不能由多个线程调用。
- `get_rune_data()` 只能由一个固定的火控发送线程调用，不能由多个线程调用。
- `process_power_rune()` 与 `get_rune_data()` 可以分别位于不同线程，并按各自频率运行。
