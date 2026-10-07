# UVC 相机内参标定

参考 `capture` / `calibrate_camera` 的离线流程，增加 `capture_uvc` 和
`calibrate_uvc`。只需 UVC 相机和棋盘格，不依赖云台、串口或 IMU。
左右相机必须分别采集、分别标定。

## 构建

依赖 OpenCV 4（含 calib3d、videoio、highgui）、fmt、yaml-cpp 和 C++17。
完整项目配置后可直接构建同名目标；没有 OpenVINO 或工业相机 SDK 时可单独构建：

```bash
cmake -S calibration -B build/uvc_calibration
cmake --build build/uvc_calibration --target capture_uvc calibrate_uvc uvc_calibration_test -j2
./build/uvc_calibration/uvc_calibration_test
```

下面均从仓库根目录运行，使用独立构建的可执行文件。

## 棋盘格与采图

修改 `configs/uvc_calibration.yaml`：`pattern_cols` / `pattern_rows` 是**内角点**列数、
行数，`center_distance_mm` 是相邻角点间距（方格边长），单位 mm。
默认 11×8 内角点、30 mm 方格，与工业相机标定配置一致。
打印时关闭页面缩放，并测量实际方格边长；使用平整、刚性的标定板。

采图读取 `configs/sentry.yaml` 中该角色的 `device`，以及共享的 UVC 分辨率、帧率、
曝光、gamma 和 gain。先确认左右设备对应关系，关闭占用相机的视觉程序。
保留实际运行时的镜头、焦距、对焦位置和分辨率；设备实际图像尺寸与配置不符时程序报错。
驱动拒绝设置控制项时会打印警告，实际读回值保存在 `capture.yaml`。

```bash
./build/uvc_calibration/capture_uvc --camera=left --output-folder=assets/uvc_calibration/left
./build/uvc_calibration/capture_uvc --camera=right --output-folder=assets/uvc_calibration/right
```

- 按 `s` 保存当前完整检测到棋盘格的原始 PNG；预览中的标记、文字和缩放不写入图片。
- 按 `q` / Esc 或关闭窗口退出。不指定输出目录时按相机角色和时间创建新目录。
- 输出目录必须为空或不存在，防止混入另一台相机或另一种分辨率的数据。
- 每台推荐 20～30 张，最低 12 张。覆盖中央、四角、边缘，以及不同距离、绕水平和
  垂直轴的倾角；每次移动后停稳再保存。连续重复同一姿态不能替代不同视角。
- 棋盘格应完整可见，保持清晰、无反光；广角边缘也要有足够大的有效角点。
- 预览检测较慢时可用 `--detect-board=false`，此时只在按 `s` 时检测。
- 采图需要图形桌面，离线标定无需显示器。

## 离线标定与镜头模型

只知道“广角”无法确定投影模型。默认用同一批角点分别拟合两种模型：

```bash
./build/uvc_calibration/calibrate_uvc assets/uvc_calibration/left --model=both
./build/uvc_calibration/calibrate_uvc assets/uvc_calibration/right --model=both
```

程序枚举目录第一层的 PNG/JPEG/BMP/TIFF，文件编号可以不连续。
角点检测使用 OpenCV `findChessboardCornersSB` 的亚像素结果；不可读或未完整检测的
图片会记录拒绝原因，不会按重投影误差自动删除样本。混合分辨率直接报错。

结果默认保存到输入目录下的 `results/`，也可用 `--output-folder=...` 指定新目录：

| 文件/目录 | 内容 |
| --- | --- |
| `pinhole.yaml` | 普通透视模型；`distort_coeffs` 顺序为 `[k1,k2,p1,p2,k3]` |
| `fisheye.yaml` | OpenCV 鱼眼模型；`distort_coeffs` 顺序为 `[k1,k2,k3,k4]` |
| `*_previews/0001.png` 等 | 左侧原始图，右侧去畸变图，按 YAML 的 `views` 顺序对应 |

YAML 包含 `camera_model`、原始 `image_width/image_height`、按行展开的
`camera_matrix`、`distort_coeffs`、整体与逐图 RMS 重投影误差（px）、实际使用的角点、
拒绝图片及原因、标定板尺寸和采图元数据。普通模型估计 k3，不沿用窄视场工业相机的
`CALIB_FIX_K3`；鱼眼模型固定 skew 为零。

预览输出保持原始分辨率，使用 `alpha/balance=1` 的新透视内参，黑边表示无对应原始
像素；`preview_camera_matrix` 仅对应预览。映射使用线性插值，不覆盖原始图片。
无需预览可传 `--previews=false`。不根据训练 RMS 自动选模型：还应检查边缘直线是否
恢复，以及未用于标定的图片表现。小 RMS 不能排除采图覆盖不足或过拟合。
某一模型失败时继续尝试另一模型，打印具体错误并以非零状态退出；已成功输出的结果保留。

鱼眼标定出现病态矩阵或外参初始化失败时，补充清晰且姿态变化充分的图片，检查棋盘格
尺寸；不要单纯关闭条件检查来让程序通过。

## 与 sentry_multithread 的关系

这里输出的是每台 UVC 的**内参**，不包含 UVC 相对云台的安装旋转、平移或时间同步。
结果文件不能覆盖 `sentry.yaml` 中工业相机的顶层 `camera_matrix/distort_coeffs`。
当前 `Decider::delta_angle()` 仍使用 FOV 线性估计，不会自动加载这些文件。
后续接入需按相机角色加载正确模型：

```text
原始检测中心像素
  → pinhole: cv::undistortPoints / fisheye: cv::fisheye::undistortPoints
  → 相机三维视线
  → UVC 到云台的安装旋转
  → 云台/世界方位角（匹配该图像时刻的姿态）
```

处理原图上的检测点时，使用原始标定内参，不使用 `preview_camera_matrix`；
鱼眼的四个畸变系数也不能传给普通模型的去畸变接口。

## 验证范围

`uvc_calibration_test` 检查自定义棋盘格检测、空白图拒绝、普通与鱼眼模型的已知参数
恢复、未见过姿态的投影、逐图与整体 RMS 一致性，以及去畸变映射和样本不足检查。
测试数据是合成数据，不能替代实机标定精度检查。
