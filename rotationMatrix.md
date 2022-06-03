# Matlab中旋转矩阵和若干函数理解说明
崔星星 2022.6.2

>matlab中众多工具箱都有涉及到旋转矩阵，欧拉角，四元数等的转换，但目前最新版本2022a中各个工具箱还不完全统一明确，这里以**通用习惯**进行表述一些常用的操作。

本文默认都是以右手坐标系，extrinsic rotation/point rotation，点坐标以列向量形式在旋转矩阵右侧相乘的惯例进行，除非特别说明。根据`rotx`函数文档，点绕x,y,z坐标轴旋转对应的旋转矩阵分别如下：<br>
![rotation_matrix](images/Rotation_matrix.png)<br>
比如空间点$p1(x_1,y_1,z_1)$绕z轴旋转$\theta$度得到$p2(x_2,y_2,z_2)$,则数学上表示为：<br>

$$\left\lbrack \begin{array}{c}
x_2 \\
y_2 \\
z_2 
\end{array}\right\rbrack =\left\lbrack \begin{array}{ccc}
\cos \left(\theta \right) & -\sin \left(\theta \right) & 0\\
\sin \left(\theta \right) & \cos \left(\theta \right) & 0\\
0 & 0 & 1
\end{array}\right\rbrack*\left\lbrack \begin{array}{c}
x_1 \\
y_1 \\
z_1 
\end{array}\right\rbrack$$


若空间点$p1(x_1,y_1,z_1)$绕多个轴旋转，比如先绕x轴旋转$\alpha$，然后绕y轴旋转$\beta$,最后绕z轴旋转$\gamma$得到$p2(x_2,y_2,z_2)$,则公式应为：$p2 = R_z(\gamma)*R_y(\beta)*R_x(\alpha)*p1$，注意它们顺序依次向左写，不能搞反！

## eul2rotm函数原理
`eul2rotm`同`rotx`,`roty`,`rotz`函数得到的旋转矩阵形式与上述是统一一致的，但要注意顺序问题，官方文档并未阐述清晰，比如`eul2rotm`函数第二个参数'sequence'指定顺序为'XYZ'，则代表$rotx(\theta)*roty(\theta)*rotz(\theta)$,但我们很容易错误理解为点先绕x轴旋转，其次绕y轴旋转，最后绕z轴旋转的旋转矩阵。比如下面示例展示出其原理，`myR1`和`myR2`数值上相等。
```matlab
thetaRadians = rand(1,3);
thetaDegrees = rad2deg(thetaRadians);
myR1 = eul2rotm(thetaRadians,'XYZ');
myR2 = rotx(thetaDegrees(1))*roty(thetaDegrees(2))*rotz(thetaDegrees(3));
err = myR1-myR2
```
```text
err =
    0.1830   -0.5948         0
    0.5568   -0.2064    0.0000
    0.5018    0.4459         0
```

## rigid3d 函数
computer vision toolbox与其他工具箱旋转矩阵表述不同，比如这个rigid3d函数，在vSLAM中表示姿态（Location和Orientation），用的非常广泛。该函数对象包含2个属性，即RotationMatrix和Translation,分别对应Orientation和Location，它们数组大小必须是3×3、1×3。注意：这个rigid3d函数的旋转矩阵与上述**通用理解形式**互为转置关系,其齐次矩阵T为4×4的矩阵，形式如下：<br>

$$T=\left\lbrack \begin{array}{cc}
R_{3\times 3}  & 0\\
t_{1\times 3}  & 1
\end{array}\right\rbrack$$
若涉及多个连续刚性变换，则坐标变换应该依次右乘，齐次矩阵T可同时进行旋转和平移变换，方便计算。