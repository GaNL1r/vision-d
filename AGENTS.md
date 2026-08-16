# Repository Guidelines

## 项目结构与模块组织

本仓库是基于 C++17 的机器人视觉项目。`src/` 存放各兵种和调试程序的入口；`tasks/` 按 `auto_aim`、`auto_buff`、`omniperception` 划分算法模块；`io/` 封装相机、串口、云台、CAN 与可选 ROS2 通信；`tools/` 提供日志、数学、滤波和线程工具。`calibration/` 包含标定程序，`tests/` 包含可独立运行的测试程序，`configs/` 保存机器人 YAML 配置，`assets/` 保存模型及演示数据。新增模块时同步更新相应目录或根目录的 `CMakeLists.txt`。

## 构建、测试与开发命令

项目主要面向 Ubuntu 22.04，依赖 OpenCV、Eigen、fmt、spdlog、yaml-cpp、OpenVINO、Ceres 及相机 SDK。配置前确认这些依赖能被 CMake 找到。

```bash
cmake -S . -B build
cmake --build build -j"$(nproc)"
cmake --build build --target auto_aim_test
./build/auto_aim_test
```

前两条命令配置并构建全部目标；后两条仅构建并运行默认演示测试。CMake 当前固定查找 `/opt/intel/openvino_2024.6.0`。`sentry_multithread` 始终生成；无需导航发布时可配置 `-DSRM_VISION_ENABLE_ROS2=OFF`。

## 编码风格与命名约定

提交前使用 `clang-format -i <file.cpp>`。仓库采用 Google 风格、2 空格缩进、100 列上限及换行大括号。类和结构体使用 `PascalCase`，函数、变量、文件及命名空间使用 `snake_case`，私有成员以 `_` 结尾。头文件使用 `.hpp`，实现使用 `.cpp`；保持系统、第三方和项目头文件分组清晰。

## 测试指南

当前没有 GTest、CTest 或覆盖率门槛；`tests/*_test.cpp` 均由 CMake 构建为独立可执行文件。新增测试需遵循 `<feature>_test.cpp` 命名，并注册、链接对应目标。优先运行 `auto_aim_test`、`detector_video_test` 或 `planner_test_offline` 等离线测试；涉及相机、串口、CAN 或云台的测试，应在 PR 中注明所需硬件、配置文件和验证结果。

## 提交与合并请求

历史主题通常简短明确，并可使用 `[feat]`、`[fix]`、`[init]` 等前缀。每个提交聚焦一个逻辑变更。PR 应说明目的、影响模块、构建/测试命令及结果，并关联相关 issue；视觉输出变化附截图或日志，模型与 YAML 变更说明适用机器人及兼容性。不要提交 `build/`、日志、录制视频或设备生成目录。
