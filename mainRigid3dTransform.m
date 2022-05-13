%% 演示三维空间中两个坐标系下的坐标点转换关系

%% 已知世界坐标系和载体坐标系，求世界坐标系下的点在载体坐标系下的坐标
absolutePose = rigid3d();% 世界坐标系姿态
carrierPose = rigid3d([0,1,0;-1,0,0;0,0,1],[10,20,30]);% 原点在[10,20,30],绕世界坐标系的Z轴旋转pi/2弧度的载体坐标系姿态
pts = [20,30,40; % 世界坐标系下的测试点
    10,20,30;
    10,10,10];
tformR = absolutePose.Rotation*carrierPose.Rotation';
tformT = (absolutePose.Translation-carrierPose.Translation)*carrierPose.Rotation';
transfom = rigid3d(tformR,tformT);% 也可以使用invert函数验证其反向变换
destPts = transformPointsForward(transfom,pts) % 载体坐标系下的坐标


%% 已知世界坐标系和载体坐标系，求载体坐标系下的点在世界坐标系下的坐标
absolutePose = rigid3d();% 世界坐标系姿态
carrierPose = rigid3d([0,1,0;-1,0,0;0,0,1],[10,20,30]);% 原点在[10,20,30],绕世界坐标系的Z轴旋转pi/2弧度的载体坐标系姿态
pts = [10,20,30; % 载体坐标系下的测试点
    0,0,0;
    10,10,10];
transfom = rigid3d(carrierPose.T*absolutePose.T);
destPts = transformPointsForward(transfom,pts) % 世界坐标系下的坐标