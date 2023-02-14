# rmoss_mindvision_driver

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build and Test (Humble)](https://github.com/robomaster-oss/rmoss_mindvision_driver/actions/workflows/ci.yml/badge.svg?branch=humble)](https://github.com/robomaster-oss/rmoss_mindvision_driver/actions/workflows/ci.yml)


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

**注意：请安装 `git-lfs` (此仓库使用git-lfs)存储较大的二进制文件**

`git-lfs`安装方式: ```apt-get install git-lfs```

```bash
cd <rmoss_ws>/src
git clone https://github.com/robomaster-oss/rmoss_interfaces.git
git clone https://github.com/robomaster-oss/rmoss_core.git
git clone https://github.com/robomaster-oss/rmoss_mindvision_driver.git
cd ..

rosdep install -y -r -q --from-paths src --ignore-src --rosdistro humble

colcon build --symlink-install
```

> 参照`rmoss_core/rmoss_cam`工具包中定义

launch方式运行：

```bash
ros2 launch rmoss_mindvision_driver mindvision_cam.launch.py
```

* 参数文件(`config/cam_params.yaml`)
* 相关相机配置(`config/*`)