---
layout: project
title: Quadrotor
permalink: /quadrotor/
---
<p align="center">
<br />
  <h3 align="center">基于室内定位系统的四轴飞行器表演</h3>
  <p align="center">
    Jinyao Zhu* and Lin-Zhuo Chen*
    <br />
    <a href="https://github.com/JinyaoZhu/STM32F4-Quad" target="_black">[Code_1]</a>
    <a href="https://github.com/LinZhuoChen/Quadrotor" target="_black">[Code_2(no os) <i class="fab fa-github"></i>]</a>
    <br /> 
  </p>
</p>

<style type="text/css">
img{text-align: center; margin: 0 auto;}
</style>

## Abstract
<body>
    <p style="text-align:justify; text-justify:inter-ideograph">
    这个项目构建了一个基于室内定位系统的四轴飞行器。该项目可以实现四轴飞行器的自主飞行，包括悬停，路径跟踪，以及相应的应用(例如乐器演奏)。
    整个系统的组成如下图所示：

</p>
</body>

<div style="text-align:center" markdown="1">
![System overview](/images/system.png "System overview")
</div>



整个系统分为以下几个部分：
1. **室内定位系统**：室内定位系统由两个红外摄像机组成，通过双目视觉原理感知四轴飞行器的坐标。
2. **状态估计器**：构建扩展卡尔曼滤波器(EKF), 融合惯导传感器(MPU6050)以及室内定位系统的观测值，估计四轴飞行器的各种状态值(坐标，速度，加速度，姿态，角速度等)
3. **位移以及姿态控制**：构建串级PID控制器，依据四轴飞行器的状态值，以及四轴飞行器的动力学方程，对四轴飞行器的位置以及姿态进行控制。



## 方法
### 1. 室内定位系统
我们在四轴飞行器上放置了三个标记点（红外LED），便于室内定位系统进行飞行器的定位。室内定位系统获取标记点坐标的流程如下：
1. 获取摄像机的内参数。
2. 估计标记点图像坐标与地面坐标的变换关系(齐次变换)。
3. 测量摄像机的空间坐标。
4. 通过两个摄像机与标记点地面坐标的连线，确定标记点的空间坐标。

<div style="text-align:center" markdown="1">
![System overview](/images/camera_2.png "System overview")
</div>

### 2. 状态估计器
#### 1. 姿态估计
四轴飞行器的姿态可以由以下两种传感器数据得到：加速度计和陀螺仪。我们根据陀螺仪
测量到的角速度建立状态转移方程, 依据加速度计获得的角度建立观测方程。
令状态量为：


$$
  x = \left(
  \begin{array}{c}
  \Omega\\
  b_w
  \end{array}
    \right)
  = \left(
  \begin{array}{c}
  \phi\\
  \theta\\
  \phi \\
  b_{w,x} \\
  b_{w,y} \\
  b_{w,z}
  \end{array}
    \right)
$$

我们建立起**状态转移方程**：

$$
x_k = Fx_{k-1} + Gu_{k-1} + w_{k-1},
$$

其中：


$$
F = \left(
  \begin{array}{c}
  1 & 0 & 0 & -dt & 0  &0  \\
  0 & 1 & 0 &  0  & -dt &0 \\
  0 & 0 & 1 &  0  &  0 &-dt \\
  0 & 0 & 0 &  1  &  0 & 0 \\
  0 & 0 & 0 &  0  &  1 & 0 \\
  0 & 0 & 0 &  0  &  0 & 1 
  \end{array}
    \right), 
G = \left(
  \begin{array}{c}
  dt \\
  dt \\
  dt \\
  0 \\
  0 \\
  0
  \end{array}
    \right),
u = \left(
  \begin{array}{c}
  \omega_{meas, x} \\
  \omega_{meas, y} \\
  \omega_{meas, z} \\
  0 \\
  0 \\
  0
  \end{array}
    \right)
$$


其中$\omega_{meas}$为陀螺仪的角速度测量值。

同时，观测量如下：


$$
  y = \left(
  \begin{array}{c}
  \Omega_{acc}
  \end{array}
    \right)
  = \left(
  \begin{array}{c}
  \phi_{acc}\\
  \theta_{acc}
  \end{array}
    \right)
$$


构建**观测方程**：



$$
y = Hx + v = \left(
  \begin{array}{c}
  1 & 0 & 0 & 0 & 0  &0  \\
  0 & 1 & 0 &  0  & 0 &0 \\
  \end{array}
    \right) 

  
  \left(
  \begin{array}{c}
  \phi\\
  \theta\\
  \phi \\
  b_{w,x} \\
  b_{w,y} \\
  b_{w,z}
  \end{array}
    \right)
  + 
  
  \left(
  \begin{array}{c}
  v_{acc}\\
  v_{acc}
  \end{array}
    \right)  
$$

在构建完状态转移方程与观测方程后，我们将其代入公式，计算出卡尔曼增益$K$，最后的更新方程如下：

$$
x_{k}^+ = x_{k}^- + K_k(y_k - Hx_{k}^-)
$$


#### 2. 位移估计

我们令位移状态量为：


$$
x = \left(
  \begin{array}{c}
  r\\
  \dot{r}\\
  b_{\ddot{r}}
  \end{array}
    \right)
$$


其中$r, \dot{r}, b_{\ddot{r}}$为飞行器的位移，速度，以及加速度测量值的零偏。


**状态转移方程**为：


$$
x_k = Fx_{k-1} + Gu_{k-1} + w_{k-1},
$$


其中：


$$
F = \left(
  \begin{array}{c}
  1 & dt & -\frac{1}{2}dt^2 \\
  0 & 1 & -dt \\
  0 & 0 & 1  
  \end{array}
    \right), 
G = \left(
  \begin{array}{c}
  \frac{1}{2}dt^2 \\
  dt \\
  0 
  \end{array}
    \right),
u = \ddot{r_{meas}}
$$

测量值为室内定位系统返回的对应坐标轴的位移信息：


$$
y = r_{cam}
$$


测量方程为：


$$
y = Hx + v_{r, cam}
$$


其中：


$$
H = \left(
  \begin{array}{c}
  1 & 0 & 0 
  \end{array}
    \right),
$$


在构建完状态转移方程与观测方程后，我们将其代入公式，计算出卡尔曼增益$K$，最后的更新方程如下：

$$
x_{k}^+ = x_{k}^- + K_k(y_k - Hx_{k}^-)
$$


### 3. 控制

#### 四轴飞行器的动力学方程

我们设飞行器的机体坐标系为$\Gamma_b$, 参考坐标系为$\Gamma_w$。飞行器具有六个自由度，包括旋转和位移。本项目以$Z-X-Y$次序的欧拉角来描述四轴飞行器的姿态：


$$
\Omega = 
\left(
 \begin{matrix}
   \varphi	\\
   \theta	\\
   \psi 
  \end{matrix} 
\right) \tag2
$$


其中$\varphi, \theta, \psi$分别表示四轴飞行器的横滚角，俯仰角和航偏角。

因此我们可以得到$\Gamma_b$到$\Gamma_w$的旋转矩阵$R_b^w$:


$$
R_w^b = 
\left(
\begin{matrix}
   c\psi c\theta & c\psi s\theta s\varphi - s\psi c\varphi& c\psi s\theta c\varphi + s\psi s\varphi	\\
   s\psi c\theta & s\psi s\theta s\varphi + c\psi c\varphi& s\psi s\theta c\varphi - c\psi s\varphi	\\
   -s\theta & c\theta s\varphi &  c\theta c\varphi
  \end{matrix}
  \right) \tag3
$$


飞行器的位移以$r$来表示：


$$
r = 
\left(
 \begin{matrix}
   r_x	\\
   r_y	\\
   r_z
  \end{matrix} 
\right) \tag4
$$


分别表示其参考坐标系下的$x,y,z$坐标。

飞行器位移的运动方程可以通过牛顿方程来描述：


$$
\ddot{r} =R_b^w \left(
 \begin{matrix}
   0	\\
   0	\\
   F 
  \end{matrix} 
\right) - \left(
 \begin{matrix}
   0	\\
   0	\\
   g 
  \end{matrix} 
\right) \tag5
$$


欧拉方程为:


$$
I\dot{\omega} = M - \omega \times I\omega  \tag6
$$


即:


$$
\dot{\omega} = I^{-1}(M - \omega \times I\omega) \tag7
$$


其中$I$ 为四轴飞行器的转动惯量，再由欧拉角的定义，可得欧拉角微分$\dot{\Omega}$与机体角速度的关系:


$$
\dot{\Omega} =\left(
 \begin{matrix}
   \dot{\varphi}	\\
   \dot{\theta}	\\
   \dot{\psi}
  \end{matrix} 
\right)   =\left(
 \begin{matrix}
   1 & tan(\theta)sin(\varphi) & tan(\theta)cos(\varphi)	\\
   \dot{\theta}	& cos(\varphi) & -sin(\varphi)\\
   \dot{\psi} & sec(\theta)sin(\varphi) & sec(\theta)cos(\varphi)
  \end{matrix} 
\right) \omega = W\omega \tag8
$$


 因此，飞行器的非线性状态方程总结如下:


$$
x =\left(
 \begin{matrix}
   r	\\
   \dot{r}	\\
   \Omega \\
   \omega \\
  \end{matrix} 
\right), 
\dot{x}=\left(
 \begin{matrix}
   \dot{r}	\\
   R_b^w\left(
 \begin{matrix}
   0	\\
   0	\\
   F 
  \end{matrix} 
\right) - \left(
 \begin{matrix}
   0	\\
   0	\\
   g 
  \end{matrix} 
\right) 	\\
   W\omega \\
   I^{-1}(M - \omega \times I\omega) 
  \end{matrix} 
\right) \tag9
$$


#### 四轴飞行器的PID控制
飞行器的控制部分主要分为：姿态控制与位移控制。位移控制器根据目标位移，速度以及加速度计算出飞行器的目标态，通过改变飞行器的姿态来产生相应的线加速度，从而实现位移控制。姿态控制器作为飞行器飞行控制
的底层控制器。其中控制器使用PID算法。四轴飞行器的控制系统如下：

<div style="text-align:center" markdown="1">
![](/images/control.png)
</div>

通过简单分析，可得每个螺旋桨的升力$F_i$, 反力矩$M_i$与绕机体坐标轴的力矩$M$的关系如下:



$$
M =\left(
 \begin{matrix}
   M_x	\\
   M_y	\\
   M_z
  \end{matrix} 
\right) = 
\left(
 \begin{matrix}
   (-F_1-F_2+F_3+F_4)L	\\
   (-F_1+F_2+F_3-F_4)L	\\
   (-M_1+M_2-M_3+M_4)\sqrt 2L
  \end{matrix} 
\right) \tag{10}
$$



由公式(1)可得到$F_i$ 与 $M_i$之间的关系:



$$
M_i = \frac{k_M}{k_F} F_i = k_mF_i \tag{11}
$$



因此可得:


$$
M =\left(
 \begin{matrix}
   -L & -L & L & L	\\
   -L & L  & L & -L \\
   -k_m\sqrt 2L & k_m\sqrt 2L & -k_m\sqrt 2L & k_m\sqrt 2L
  \end{matrix} 
\right) \left(
 \begin{matrix}
   F_1	\\
   F_2	\\
   F_3 \\
   F_4
  \end{matrix} 
\right) \tag{12}
$$


我们令:


$$
X = \left(
 \begin{matrix}
   -L & -L & L & L	\\
   -L & L  & L & -L \\
   -k_m\sqrt 2L & k_m\sqrt 2L & -k_m\sqrt 2L & k_m\sqrt 2L
  \end{matrix} 
\right) , F =  \left(
 \begin{matrix}
   F_1	\\
   F_2	\\
   F_3 \\
   F_4
  \end{matrix} 
\right)
$$



飞行器的误差欧拉角$\Omega_{error}$为:


$$
\Omega_{error} = \Omega_{des} - \Omega
$$


飞行器的姿态控制器采用角速度PD控制， 结构如下:

<div style="text-align:center" markdown="1">
![](/images/attitude.png)
</div>

飞行器的位移控制主要分为两步，一是位移的PID控制，二是目标加速度到角速度的转换，位移控制器结构如下:


<div style="text-align:center" markdown="1">
![](/images/position.png)
</div>

其中:


$$
\varphi_{des}=asin(-\frac{\ddot{r_{des,y}}}{g}) \\
\theta_{des}=asin(\frac{\ddot{r_{des,x}}}{gcos(\varphi)})
$$

## Video (Youtube)
<div class="embed-container">
  <iframe
      src="https://www.youtube.com/embed/XSEBEPnCcRU"
      width="700"
      height="480"
      frameborder="0"
      allowfullscreen="">
  </iframe>
</div>