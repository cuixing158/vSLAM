# 记忆泊车项目（单目SLAM）

> 单目SLAM应用3D UE仿真数据用于记忆泊车项目，持续不定期更新。

## record log
1. 2022.4.13,其中具有工程落地方面的算法有2020年的MAGSAC++,具有较强的鲁棒估计，最新版本opencv已集成此算法，ubuntu上郭倩可以尝试此算法，此算法开源地址为：https://github.com/danini/magsac <br>
1. 2022.4.13,由于我们场景停车场无闭环检测，仅单目摄像头，这样对3D重建有一定的局限性，更多尝试Sturct from motion topic，借鉴vSLAM方法，重点特征点检测/匹配/鲁棒估计。这里重要文献是“2016_struct from motion revisited.pdf”，开源库/软件是https://demuc.de/colmap/ 
1. 2022.5.6 
使用orb特征检测，经由鲁棒估计，内点筛选等基本步骤后，可以得到较为准确的匹配效果，见下图：<br>
![orb-match](images/orb_match.jpg)<br>
1. 2022.5.13
单目slam是无法得到实际物理尺寸的，[要通过诸于GPS或者标定物等实际物理尺寸计算出scale factor推算整个map的尺寸。](https://robotics.stackexchange.com/questions/22192/scale-factor-of-monocular-slam-simultaneous-localization-and-mapping)
根据曾总的图片和ground Truth,在初步跑通的整个流程下的估计地图如下：<br>
![orb-slam2](images/orb-slam2.jpg)<br>
1. 2022.5.22,尺度由全局地图尺度改为累计尺度中值进行处理，地图更新实时加入Ground Truth来对比。
1. 2022.5.30,坐标系统,roll,pitch,yaw规则均以参考2和3中的定义进行。
1. 2022.6.1, 曾总新数据集"20220527"下在拐弯（90°）出后出现较大漂移偏差(135°)方向。
1. 2022.6.2, 曾总数据集“20220527”在隔帧选取的情况下+动态尺度范围下，算法误差迅速减小，位置误差约1m/100m.
![orb-map](images/sim20220607.jpg)<br>
1. 2022.6.7，开始研究定位算法，比如典型的Monte Carlo Locallization.
1. 2022.6.10,Monte Carlo Locallization适用于已知occupancy map地图，scan ranged data和odometry sensor data. 待温故bag of visual words，看如何应用到记忆泊车。
1. 2022.6.16,待加入基于bagOfFeatures的场景识别,看看优化图的效果。
1. 2022.6.21 研究了鱼眼镜头根据矫正表去畸变的过程，完成了C++和Matlab的实现。留下的问题：一维插值如何使用C++高效的实现（插值点远大于样本点），要求类似matlab函数interp1一样高效。
1. 2022.6.29 受曾总指示，着手以前同事的2D建图，看如何融于到目前郭倩，徐庆华的3D SLAM中来。
1. 2022.7.9 受曾总指示，帮他把simulink中3D仿真的数据转为jpg,csv可供C++可读程序，见MatFcnProgram.m和SimUEdata.m文件。


## 相机标定内外参
- **相机内参**
 对公司常用的桌面usb “HD Camera”新摄像头1929×1080分辨率图像，棋盘网格每个格子大小为3.96cm，标定内参矩阵intrinsicMatrix为（OpenCV格式）：<br>

| 795.0610     | 0 | 966.1896    |
| :----:       |    :----:   |   :----: |
| 0      | 811.7286      | 547.8711   |
| 0   | 0        | 1.0000      |

径向畸变系数k1,k2：
`0.000052311553816036739269, -0.030161743407807767997`
标定误差：<br>
![img](images/calib.jpg)

- **相机外参（相机到车辆坐标系的3D刚性转换矩阵）：**<br>
外参是一个根据相机和车辆坐标系依赖的数据，根据最新的曾总仿真数据计算得到的
旋转矩阵$R=$

|0.999999996770861 |  0.000080363190730 |  0.000000189317968 |
| :----:       |    :----:   |   :----: |
|-0.000080362728065 |   0.999998421301325 |  -0.001775087797789|
|-0.000000331969389 |   0.001775087776843 |   0.999998424530396|

平移向量$T=$
`-0.009840385015821 ,0.010908616135787 ,-1.579502896883899`

## 疑惑点
1. 如何理解各个map point的景深？即景深范围与orb尺度的关系？
1. helperTrackLastKeyFrame.m文件函数中estimateWorldCameraPose使用这个来获取当前相机姿态，能否使用relativeCameraPose函数来代替（平移向量归一化处理）？
1. 为何要剔除mapPoint/landmarks？难道是为了减少存储量还是计算量？
1. 在实施定位步骤之前，建的地图以什么形式保存，保存哪些关键信息？

## Evaluation
对评估的轨迹路径和真值路径指标为ape(absolute pose error)和rpe(relative pose error)

## Reference
1. [Useful tools for the RGB-D benchmark](https://vision.in.tum.de/data/datasets/rgbd-dataset/tools)<br>
1. [Coordinate Systems in Automated Driving Toolbox](https://ww2.mathworks.cn/help/driving/ug/coordinate-systems.html)<br>
1. [Coordinate Systems for Unreal Engine Simulation in Automated Driving Toolbox](https://ww2.mathworks.cn/help/driving/ug/coordinate-systems-for-3d-simulation-in-automated-driving-toolbox.html)
1. [Localization Algorithms](https://ww2.mathworks.cn/help/nav/localization-algorithms.html)
1. [导航系统中里程计研究综述](https://cloud.tencent.com/developer/article/1812407)
