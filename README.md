# AkaiVision

## 引用
[海康相机ROS驱动](https://github.com/luckyluckydadada/HIKROBOT-MVS-CAMERA-ROS)

## 作者
[JiangMingyi(Akai)](https://github.com/akaimoe)

特别鸣谢：WangSikai(wsk12345)

## 环境
1. ubuntu == 20.04
2. OpenCV >= 4.1.1
3. HikVision Driver (MVS)
4. spdlog
   ``` shell
	git clone https://github.com/gabime/spdlog.git
	cd spdlog && mkdir build && cd build
	cmake .. && make -j
	make install
   ```
5. Ceres-Solver 
   ``` shell
	# CMake
	sudo apt-get install cmake
	# google-glog + gflags
	sudo apt-get install libgoogle-glog-dev libgflags-dev
	# BLAS & LAPACK
	sudo apt-get install libatlas-base-dev
	# Eigen3
	sudo apt-get install libeigen3-dev
	# SuiteSparse and CXSparse (optional)
	sudo apt-get install libsuitesparse-dev
	
	git clone https://ceres-solver.googlesource.com/ceres-solver
	cd ceres-solver && mkdir ceres-bin && cd ceres-bin
	cmake -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF ..
	make -j3
	make install
   ```
## 部署	
``` shell
git clone git@github.com:akaimoe/AkaiVisionROS.git
catkin_make
catkin_make
```
然后就可以通过launch文件以来运行程序
``` shell
roslaunch launch/AkaiVision.launch
```
## 运行
如果只是想运行视觉识别程序，可直接运行Armoret节点的YoloDateMain
``` shell
cd AkaiVision
catkin_make
source devel/setup.bash 
rosrun ArmorDet YoloDateMain 
```
如果想运行完整的程序（包括自动巡航），请下载[Akiemoe_Lio_Navigation](https://github.com/akaimoe/Akiemoe_Lio_Navigation)，并确保它与AkaiVisionROS位于同一路径。此导航程序基于[Fastlio2](https://github.com/hku-mars/FAST_LIO)完成

## Tips
1. 如果串口出现无法读写的问题，可尝试运行一下指令
``` shell
sudo usermod -aG dialout your_username
```
2. 如果使用外接的ROS IMU，自行替换订阅发布内容并修改IMU到相机的旋转矩阵（PoseSolver/include/Constants.hpp中修改)即可
3. 若使用其他型号的相机，自行替换camera节点并修改订阅内容即可
4. 串口通讯协议可根据自身需求在serialdriver节点修改

## TODO
最近玩了一下YOLOV8，感觉爽的一批，等啥时候有时间了更新一下项目里面的识别模型  2024/5/13

## 问题
若有问题或建议~~或者来找我玩原神~~，可直接发issues或者通过以下方式来联系我： [![](https://img.shields.io/badge/Telegram-Akai%20moe-white?style=flat&logo=telegram)](https://t.me/akai_moe) 


