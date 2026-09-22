# 第三方依赖与许可证

本包不内嵌第三方源码。

| 依赖 | 来源 | 许可证 | 用途 |
| --- | --- | --- | --- |
| ROS 2 Humble（rclcpp、消息包、tf2） | https://github.com/ros2 | Apache-2.0 / 各包声明 | ROS 接口与 TF |
| Eigen 3 | https://eigen.tuxfamily.org | MPL-2.0 | 固定尺寸线性代数与刚体变换 |
| OpenCV 4 | https://opencv.org | Apache-2.0 | 低频 ORB 特征提取与 Hamming 匹配 |

明确不使用或链接 cuVSLAM、CUDA、OpenCL。OpenCV 不参与逐帧稠密 ICP、LK 光流或自研优化器，仅用于有界频率的回环重识别。
