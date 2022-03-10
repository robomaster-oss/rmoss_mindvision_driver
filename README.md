# rmoss_mindvision_driver

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build and Test (Galactic)](https://github.com/robomaster-oss/rmoss_mindvision_driver/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/robomaster-oss/rmoss_mindvision_driver/actions/workflows/ci.yml)


![](rmoss_bg.png)

## 简介

`rmoss_mindvision_driver`提供了适配了`rmoss_cam`的MindVision USB3.0工业相机驱动，并基于`rmoss_cam`实现相机ROS节点。

文件说明：

* `mindvision_cam.hpp.hpp/cpp`：MindVision相机设备实现。
* `mindvision_cam_node.hpp/cpp`: ROS顶层模块（基于`MindVisionCam`和`CamServer`），实现MindVision相机节点。

## 依赖

* `rmoss_interfaces`
* `rmoss_cam`
* `OpenCV 4.x`

## 使用方式

> 参照`rmoss_core/rmoss_cam`工具包中定义

launch方式运行：

```bash
ros2 launch rmoss_mindvision_driver mindvision_cam.launch.py
```

* 参数文件(`config/cam_params.yaml`)
* 相关相机配置(`config/*`)