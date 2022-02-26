# rmoss_entity_cam模块

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

![](rmoss_bg.png)

## 简介

`rmoss_entity_cam`提供了大恒和MindVision USB3.0工业相机ROS节点相关功能。

文件说明：

* `daheng_cam.hpp.hpp/cpp` : 大恒相机设备实现。
* `mindvision_cam.hpp.hpp/cpp`：MindVision相机设备实现。
* `mindvision_cam_node.hpp/cpp` ，`daheng_cam_node.hpp/cpp` :  ROS顶层模块（基于`DaHengCam`,`MindVisionCam`和`CamServer`），实现大恒相机节点和MindVision相机节点。

## 依赖

* `rmoss_interfaces`
* `rmoss_cam`
* `OpenCV 4.x`

## 使用方式

> 参照`rmoss_core/rmoss_cam`工具包中定义

### 大恒相机

launch方式运行：

```bash
ros2 launch rmoss_entity_cam daheng_cam.launch.py
```

* 参数文件(`config/cam_params.yaml`)
* 相关相机配置(`config/*`)

### MindVision相机

launch方式运行：

```bash
ros2 launch rmoss_entity_cam mindvision_cam.launch.py
```

* 参数文件(`config/cam_params.yaml`)
* 相关相机配置(`config/*`)