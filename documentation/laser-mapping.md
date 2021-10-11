# A-LOAM - laser mapping 代码分析

## 简介

作为对已注释代码的补充说明，主要是进行逻辑上的梳理。

## 整体思路

laserMapping 节点的主要作用是作为 a loam 的后端建图进程对激光雷达的位姿进行相对于里程计更高精度的估计，建立并维护全局地图，大致过程为：

- 从 scanOdometry 中接收识别为角点、平面的点云，当前帧的原始点云（经过一定预处理），以及里程计估计
- 每一次从各个缓冲队列中获取一组角点、平面点以及相对于位姿消息
- 接收到里程计估计的位姿时，利用当前的修正估计对位姿进行修正并作为高频位姿估计发布
- 根据当前得到的位姿，对以及存放的点云地图进行空间上的预处理，保证新的的点云有足够的空间存放
- 对每一个特征点（平面点/角点），从全局数据中提取局部地图，从中找到可以的匹配特征点及其构成的点，对角点来说是两个点构成一个角点特征；对平面点来说是三个点构成一个平面
- 将当前分析的特征点以及找到的匹配特征点作为观测值加入 ceres 问题中构建残差
- 对所有观测值进行优化
- 更新世界坐标系下的位姿以及按照一定频率发布更新后的全局以及局部地图，还有当前帧的全分辨率点云

## 前置知识

### 地图结构及维护方法

#### 地图结构

A-LOAM 维护一个尺寸为 `w x h x d` 的三维栅格地图，分辨率为 50m，代码中三个方向的尺寸分别是 21, 21, 11，即地图在三个方向的长度分别为：1150m, 1150m, 550m。每个栅格中存放一个点云作为在该位置的地图信息（包括角点和平面点），点云中点的坐标为全局坐标坐标系下的坐标，因此只要将所有点云加在一起就能获得全局地图。同时每个点云通过体素滤波进行按照角点和平面点各自的分辨率进行降采样来保证每个点云中点的数量不会过多。

初始情况下，世界坐标系下的 `(0, 0, 0)` 对应在三维栅格地图的中心 `(ox, oy, oz)`，假设地图分辨率为 `res`，对任意一个位置 `(x, y, z)`我们可以通过这个公式来进行栅格坐标的计算：

```c++
grid_x = round(x / res) + ox
grid_y = round(y / res) + oy
grid_z = round(z / res) + oz
```

此外对坐标是负数的情况需要额外减 1，保证坐标是 `-0.5` 和 `0` 会对应不同的栅格。A-LOAM 中使用一个长度为 `w x h x d` 的数组来作为栅格地图。坐标为 `(i, j, k)` 对应数组下标为 `[i + w * j + h * k]`

#### 地图维护方法

理想情况下，地图数据应该均匀分布在栅格地图中，但实际中由于载体运动的轨迹的中心不一定（很大可能）不在初始位置（尤其是当载体往一个方向运动时），因此地图数据可能会比较密集地分布在某一方向，以一维地图为例，下面的数组表示存放的地图，非负数表示有数据，分辨率假设为 `1`，最开始只有栅格中心有数据，原点对应的栅格位置为 `4`：

```c++
x = 0
index = x / 1 + 4 = 4
map = [-1, -1, -1, -1, 0*, -1, -1, -1, -1]
```

随着载体往一个方向移动，地图开始朝某个方向积累数据：

```c++
x = 4
index = x / 1 + 4 = 8
map = [-1, -1, -1, -1, 0, 1, 2, 3, 4]
```

这个时候如果下一帧数据还是在这一个方向上积累，之后很有可能会存放不下，因此，我们需要将地图整体向左移动一个，以在右侧腾出更多的空间，同时原点对应的栅格坐标也左移了一个单位，如下所示：

```c++
map = [-1, -1, -1, -1, 0, 1, 2, 3, 4] => [-1, -1, -1, 0, 1, 2, 3, 4, -1]
ox = 4 - 1 = 3
```

此时用更新后的原点栅格坐标计算位姿的栅格坐标可以保证地图的正确性：

```c++
x = 5
index = x / 1 + 3 = 8
map = [-1, -1, -1, 0, 1, 2, 3, 4, 5]
```

可以想象的是，如果载体一直往右走，最后我们将地图往左侧移动时，将不可避免的损失一些数据，代码中没有处理这种情况，因为按照给定的能覆盖的平面范围差不多是 1.25 平方公里，大部分情况下能覆盖所有路径了。但是如果要考虑这种情况下，大概的思路可以是

- 对栅格数量进行扩充，这样可能造成消耗内存的增多
- 保持栅格数量的情况下，对超出栅格区域内的数据存储到文件中，等到载体运动到附近区域时再读取到内存中
- 保持栅格数量的情况下，降低栅格的分辨率（例如从 50m 降低至 100m），这样同样数量的栅格能够存储尺寸更大的地图，同时为了不让内存消耗太大也要将点云的下采样分辨率降低，因此还是有一部分信息的损失

A-LOAM 中维护方法比上面稍微复杂一点点，首先通过估计的载体的位姿计算出其栅格坐标，这里我们不能当其栅格坐标到最边缘时才移动地图，因为每一个位姿对应的点云范围可能很广，代码中设置缓冲区域为 3，即位姿在离边缘少于 3 个栅格对地图进行移动，这样可以保证位姿附近 150m 的点云都能够进行存放。

### 前后端坐标系

- 前端里程计中涉及两个坐标系，`/camera_init` 以及 `/laser_odom` 系，可以理解为前端中的全局世界坐标系和载体坐标系，而前端里程计的工作就是通过匹配前后两帧点云来估计载体坐标系在世界坐标系中的位姿，这个估计是高频并且精度相对不高的
- 后端建图中也涉及两个坐标系，`/camera_int` 以及 `/aft_mapped` 系，同样也可以理解为后端中的全局世界坐标系以及载体坐标系，后端的工作是通过将点云以及全局地图进行匹配来估计 `/aft_mapped` 系在 `/camera_int` 中的位姿，相对于前端频率较低，但精度较高。可以想象的是后端估计的位姿和前端估计的位姿有一定差异，因此后端中还维护一个转换矩阵（`wmap_T_odom`），用来将从前端接收到的位姿估计转换至后端估计的位姿，即对前端里程计估计的位姿进行修正

## 接收消息并进行相应处理

主要就是一系列 ROS subscribers 及其回调函数的作用，比较简单，基本上就是接收消息并存放在相应的 Buffer 中。对位姿的处理稍微特殊一点，除了将其存放至 Buffer 之外，利用当前估计的 `wmap_T_odom` 对位姿进行修正然后作为后端的高频位姿进行发布。注意这里虽然发布的位姿和前端频率一致，但由于利用了后端估计的信息，精度会相对高一点。

## 预处理

预处理分为两步，对 Buffer 进行预处理以及对地图进行预处理。

### Buffer 预处理

以角点点云 Buffer 的最早一阵数据的时间作为参考，在其他 Buffer 中筛除掉比该时间早的数据。

```c++
mBuf.lock();
// 以最旧的角点数据的时间为参考，舍弃掉过早的数据
while (!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
    odometryBuf.pop();
if (odometryBuf.empty())
{
    mBuf.unlock();
    break;
}

while (!surfLastBuf.empty() && surfLastBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
    surfLastBuf.pop();
if (surfLastBuf.empty())
{
    mBuf.unlock();
    break;
}

while (!fullResBuf.empty() && fullResBuf.front()->header.stamp.toSec() < cornerLastBuf.front()->header.stamp.toSec())
    fullResBuf.pop();
if (fullResBuf.empty())
{
    mBuf.unlock();
    break;
}
```

筛除完后从各个 Buffer 中取出一个消息组成一组数据，由于前端里程计每一次会以同一时间戳发布四个数据，理论上这一组数据时间戳应该完全一致。这里当取出完一个角点点云之后，如果 Buffer 中还有数据，则将 Buffer 清空，这样可以保证每次后端处理的都是最新的数据，有利于实时性。

```c++
// 确认收到的角点、平面、全点云以及里程计数据时间戳是否一致，由于前端每次会同时发布这四个数据，所以理论上应该可以得到四个时间戳相同的数据
if (timeLaserCloudCornerLast != timeLaserOdometry ||
    timeLaserCloudSurfLast != timeLaserOdometry ||
    timeLaserCloudFullRes != timeLaserOdometry)
{
    printf("time corner %f surf %f full %f odom %f \n", timeLaserCloudCornerLast, timeLaserCloudSurfLast, timeLaserCloudFullRes, timeLaserOdometry);
    printf("unsync messeage!");
    mBuf.unlock();
    break;
}

// 取出下一帧要处理的角点、平面以及全部点云消息，转换为 PCL 格式
laserCloudCornerLast->clear();
pcl::fromROSMsg(*cornerLastBuf.front(), *laserCloudCornerLast);
cornerLastBuf.pop();

laserCloudSurfLast->clear();
pcl::fromROSMsg(*surfLastBuf.front(), *laserCloudSurfLast);
surfLastBuf.pop();

laserCloudFullRes->clear();
pcl::fromROSMsg(*fullResBuf.front(), *laserCloudFullRes);
fullResBuf.pop();

// 取出上述消息对应的前端计算的当前位姿，存到对应的 q 和 t 中
q_wodom_curr.x() = odometryBuf.front()->pose.pose.orientation.x;
q_wodom_curr.y() = odometryBuf.front()->pose.pose.orientation.y;
q_wodom_curr.z() = odometryBuf.front()->pose.pose.orientation.z;
q_wodom_curr.w() = odometryBuf.front()->pose.pose.orientation.w;
t_wodom_curr.x() = odometryBuf.front()->pose.pose.position.x;
t_wodom_curr.y() = odometryBuf.front()->pose.pose.position.y;
t_wodom_curr.z() = odometryBuf.front()->pose.pose.position.z;
odometryBuf.pop();

// 为了提高实时性，将还没来得及处理的角点点云去除
//（同时在下一次迭代中也会在上面操作，将去除的这些角点对应的其他点云以及里程计消息清除）
while(!cornerLastBuf.empty())
{
    cornerLastBuf.pop();
    printf("drop lidar frame in mapping for real time performance \n");
}
```

### 地图预处理

根据上述提到的地图维护方法，根据当前获得的位姿信息判断是否需要移动地图（距边缘少于 3 个栅格），如果需要则进行移动。

```c++
// 计算当前位姿在全局三维栅格中存放的位置（索引），栅格分辨率为 50 
// + 25.0 起四舍五入的作用，([0, 25) 取 0, [25, 50} 进一)
int centerCubeI = int((t_w_curr.x() + 25.0) / 50.0) + laserCloudCenWidth; // 10
int centerCubeJ = int((t_w_curr.y() + 25.0) / 50.0) + laserCloudCenHeight; // 10
int centerCubeK = int((t_w_curr.z() + 25.0) / 50.0) + laserCloudCenDepth; // 5

// 对负数的请款做出调整，保证 (-1, -1 , -1) 不会和 (0, 0, 0) 存在同一位置
if (t_w_curr.x() + 25.0 < 0)
    centerCubeI--;
if (t_w_curr.y() + 25.0 < 0)
    centerCubeJ--;
if (t_w_curr.z() + 25.0 < 0)
    centerCubeK--;

// 当点云中心栅格坐标靠近栅格宽度负方向边缘时，需要将所有点云向宽度正方向移一个栅格，以腾出空间，保证栅格能够在宽度负方向容纳更多点云
// 这里用 3 个栅格长度（150m）作为缓冲区域，即保证当前位姿周围 150 m 范围内的激光点都可以进行存放
while (centerCubeI < 3)
{
    for (int j = 0; j < laserCloudHeight; j++)
    {
        for (int k = 0; k < laserCloudDepth; k++)
        {
            // 宽度方向上的边界（最后一个点云） 
            int i = laserCloudWidth - 1;
            pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k]; 
            pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            // 不改变点云在高度和深度的栅格坐标，对其在宽度方向向往正方向移动一个单位，正方向边缘的点云会被移除
            for (; i >= 1; i--)
            {
                laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
                laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                    laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
            }
            laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCubeCornerPointer;
            laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
                laserCloudCubeSurfPointer;
            laserCloudCubeCornerPointer->clear();
            laserCloudCubeSurfPointer->clear();
        }
    }

    centerCubeI++;
    // 在移动完点云后，需要将原点对应的栅格坐标也向宽度正方向移动一个单位，保证栅格坐标计算的正确性
    laserCloudCenWidth++;
}

// 当点云中心栅格坐标靠近栅格宽度正方向边缘时，需要将所有点云向宽度负方向移一个栅格，进行和上面一样的操作
while (centerCubeI >= laserCloudWidth - 3)
{ 
    // ...
}

// 对栅格高度方向进行一样的操作
while (centerCubeJ < 3)
{
    // ...
}

// 对栅格高度方向进行一样的操作
while (centerCubeJ >= laserCloudHeight - 3)
{
    // ...
}

// 对栅格深度方向进行一样的操作
while (centerCubeK < 3)
{
    // ...
}

// 对栅格深度方向进行一样的操作
while (centerCubeK >= laserCloudDepth - 3)
{
    // ...
}
```

## 点云匹配及位姿优化

主要分为 3 个步骤：局部地图提取、特征匹配及构建残差、位姿优化

### 局部地图提取

过程比较简单，将位姿对应栅格周围的栅格点云相加起来即可，记录用到的点云，之后更新时会用到，并且对待特征点点云进行降采样处理，控制计算规模：

```c++
int laserCloudValidNum = 0;
int laserCloudSurroundNum = 0;

// 获取当前位置周围（100m , 100m ,10m）的点云作为匹配对象
for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
{
    for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
    {
        for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++)
        {
            if (i >= 0 && i < laserCloudWidth &&
                j >= 0 && j < laserCloudHeight &&
                k >= 0 && k < laserCloudDepth)
            { 
                laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
                laserCloudValidNum++;
                laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
                laserCloudSurroundNum++;
            }
        }
    }
}

// 提取局部地图点云（当前载体附近的点云）
laserCloudCornerFromMap->clear();
laserCloudSurfFromMap->clear();
for (int i = 0; i < laserCloudValidNum; i++)
{
    *laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
    *laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
}
int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();


// 对待处理的角点点云进行降采样处理
pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
downSizeFilterCorner.filter(*laserCloudCornerStack);
int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

// 对待处理的平面点云进行降采样处理
pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
downSizeFilterSurf.filter(*laserCloudSurfStack);
int laserCloudSurfStackNum = laserCloudSurfStack->points.size();
```

提取完之后建立 KD 树，便于快速搜索：

```c++
// 用局部地图中的角点和平面生成 KD 树
kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);
```

### 特征匹配及构建残差

只有当角点和平面点都有一定数量时才进行优化

#### 初始化优化问题

常规操作，如下所示：

```c++
// 初始化图，损失函数，设置定鲁棒核函数
ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
ceres::LocalParameterization *q_parameterization =
    new ceres::EigenQuaternionParameterization();
ceres::Problem::Options problem_options;

// 加入要优化的参数块，这里分别是姿态（4维）和位置（3维）
ceres::Problem problem(problem_options);
problem.AddParameterBlock(parameters, 4, q_parameterization);
problem.AddParameterBlock(parameters + 4, 3);
```

#### 角点匹配及残差构建

如下所示，大致过程为，对每一个角点：

- 通过 KD 树搜索出局部地图中离这个点最近的 5 个点
- 通过特征值计算判断这 5 个点是不是位于同一直线，如果是才进行下一步
- 通过 PCA 算法进行特征值计算中可以计算出 5 个点所处直线的方向，以这 5 个点的几何中心为中点，按照计算出的直线方向在两侧各取一个点
    - 如果 5 个点属于同一直线，计算出的三个特征值中有一个特征值会明显比较大，并且该特征值对应的特征向量为直线方向
- 将取得的两个点和待匹配点传入 ceres 中构建残差，残差构建方式和论文一致，在前端也使用过，这里不再赘述

```c++
for (int i = 0; i < laserCloudCornerStackNum; i++)
{
    // 获取原始点并将其坐标转换至全局坐标系，在角点 KD 树搜索最近的 5 个点
    pointOri = laserCloudCornerStack->points[i];
    pointAssociateToMap(&pointOri, &pointSel);
    kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis); 

    // 当搜索到的 5 个点都在当前选择点的 1m 范围内，才进行以下操作
    if (pointSearchSqDis[4] < 1.0)
    { 
        // 获取这 5 个点的位置并计算其平均值作为线段中心
        std::vector<Eigen::Vector3d> nearCorners;
        Eigen::Vector3d center(0, 0, 0);
        for (int j = 0; j < 5; j++)
        {
            Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
                                laserCloudCornerFromMap->points[pointSearchInd[j]].y,
                                laserCloudCornerFromMap->points[pointSearchInd[j]].z);
            center = center + tmp;
            nearCorners.push_back(tmp);
        }
        center = center / 5.0;

        // 根据距离设置这 5 个点的协方差矩阵 cov = sum(mean * mean^T)
        Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
        for (int j = 0; j < 5; j++)
        {
            Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
            covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
        }

        // 协方差矩阵是厄米特矩阵，计算其特征值和特征向量
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

        // 如果是这 5 个点来至同一个线特征，那么最大的特征值会比另外两个都大很多，
        // 并且其相应的特征向量表明该直线的方向
        // 这里用 3 倍大小作为衡量标准
        Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
        Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
        if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
        { 
            // 在这 5 个点的中心，根据直线（特征向量）方向计算出两个点的坐标作为匹配点
            Eigen::Vector3d point_on_line = center;
            Eigen::Vector3d point_a, point_b;
            point_a = 0.1 * unit_direction + point_on_line;
            point_b = -0.1 * unit_direction + point_on_line;

            // 将两个点以及当前选择的角点（注意是在点云中的，坐标是在载体坐标系下表示）传入 ceres 中构建残差
            ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
            problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
            corner_num++;
        }
    }
```

除此之外，代码中还有一个被注释掉了的残差的构建，如下所示，大致思路是，如果这 5 个点都离待匹配点很近，则考虑它们为同一个点，将这 5 个点的中心和待匹配点传入 ceres 中，残差为它们之间的距离，估计的时候最小化这个距离：

```c++
// 如果这个 5 个点离选择的点很近，则考虑为同一个点，
// 计算中心点并将其和当前选择的点传入 ceres 中，最小化这两个点的距离
/*
else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
{
    Eigen::Vector3d center(0, 0, 0);
    for (int j = 0; j < 5; j++)
    {
        Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
                            laserCloudCornerFromMap->points[pointSearchInd[j]].y,
                            laserCloudCornerFromMap->points[pointSearchInd[j]].z);
        center = center + tmp;
    }
    center = center / 5.0;	
    Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
    ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
    problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
}
*/
```

#### 平面点匹配及残差构建

如下所示，大致过程和角点差不多，对每一个平面点：

- 通过 KD 树搜索出局部地图中离这个点最近的 5 个点
- 判断这 5 个点是不是位于同一平面，如果是才进行下一步，判断是否平面有两个方法，代码中选择了第二种
  - 同样进行 PCA 并进行特征计算，如果是平面则其中有一个特征值会比较小，其对应的特征向量为平面法向量，论文中用的是这个方法
  - 平面拟合，假定这 5 个点属于同一平面，进行平面拟合，对于一个平面，它的方程为 `Ax + By + Cz + D =0`，将常数项化为 1 有： `A'x + B'y + C'z + 1 =0`，法向量为 `(A', B', C')`，因此通过 5 个点建立方程组，利用最小二乘法解出方程系数即可求得法向量
- 将待匹配点，计算出的法向量以及它的模的倒数传入 ceres 中构建残差，残差计算方式也是通过法向量和传入的常数项计算该点到平面的距离，估计过程中最小化该距离


```c++
// 对将要匹配中的每个平面点进行以下操作
int surf_num = 0;
for (int i = 0; i < laserCloudSurfStackNum; i++)
{
    // 获取原始点并将其坐标转换至全局坐标系，在平面点的 KD 树搜索最近的 5 个点
    pointOri = laserCloudSurfStack->points[i];
    pointAssociateToMap(&pointOri, &pointSel);
    kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

    Eigen::Matrix<double, 5, 3> matA0;
    Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
    // 当搜索到的 5 个点都在当前选择点的 1m 范围内，才进行以下操作
    if (pointSearchSqDis[4] < 1.0)
    {
        // 将 5 个点按列插入 5x3 矩阵中
        for (int j = 0; j < 5; j++)
        {
            matA0(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
            matA0(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
            matA0(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
        }
        // 计算平面法向量，平面方程为 Ax + By + Cz = -1，代入 5 个点，利用最小二乘法解出参数即可得到法向量
        Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
        double negative_OA_dot_norm = 1 / norm.norm();
        norm.normalize();

        // Here n(pa, pb, pc) is unit norm of plane
        // 通过计算每个点到平面的距离来判断平面拟合的效果，距离公式为：d = |Ax + By + Cz + D|/sqrt(A^2+B^2+C^2)
        // 归一化后平面公式为：Ax + By + Cz + D = 0, D = 1/sqrt(A'^2+B'^2+C'^2)
        // 因此，计算公式为：d = Ax + By + Cz + negative_OA_dot_norm
        bool planeValid = true;
        for (int j = 0; j < 5; j++)
        {
            // if OX * n > 0.2, then plane is not fit well
            if (fabs(norm(0) * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
                        norm(1) * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
                        norm(2) * laserCloudSurfFromMap->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
            {
                planeValid = false;
                break;
            }
        }
        Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
        if (planeValid)
        {
            // 将选择的点和法向量传入 ceres 中构建残差块
            ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
            problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
            surf_num++;
        }
    }
```

和角点一样，除上述步骤之外，代码中还有一个被注释掉了的残差的构建，这里不再赘述。

### 位姿优化

建立完所有残差块之后，进行优化。

```c++
// 对整个图（包括平面点和角点）进行优化
TicToc t_solver;
ceres::Solver::Options options;
options.linear_solver_type = ceres::DENSE_QR;
options.max_num_iterations = 4;
options.minimizer_progress_to_stdout = false;
options.check_gradients = false;
options.gradient_check_relative_precision = 1e-4;
ceres::Solver::Summary summary;
ceres::Solve(options, &problem, &summary);
printf("mapping solver time %f ms \n", t_solver.toc());
```

## 位姿及地图更新

利用优化后的全局位姿，更新相对于里程计位姿的转换（修正量）：

```c++
void transformUpdate()
{
	// 利用后端估计的世界位姿以及前端估计的世界位姿，更新位姿修正需要的旋转和平移
	q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
	t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
}
```

利用更新后的位姿，将当前特征点云中的所有点转换至世界坐标系下，并计算出栅格位置，插入到对应的点云中，对点云更新之后重新进行降采样处理控制点云规模：

```c++
for (int i = 0; i < laserCloudCornerStackNum; i++)
{
    // 对当前新加入的角点点云中的所有点，进行坐标转换至世界坐标系下，找到其对应的点云栅格坐标
    // 插入到该栅格中存放的点云中
    pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);

    int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
    int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
    int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

    if (pointSel.x + 25.0 < 0)
        cubeI--;
    if (pointSel.y + 25.0 < 0)
        cubeJ--;
    if (pointSel.z + 25.0 < 0)
        cubeK--;

    if (cubeI >= 0 && cubeI < laserCloudWidth &&
        cubeJ >= 0 && cubeJ < laserCloudHeight &&
        cubeK >= 0 && cubeK < laserCloudDepth)
    {
        int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
        laserCloudCornerArray[cubeInd]->push_back(pointSel);
    }
}

for (int i = 0; i < laserCloudSurfStackNum; i++)
{
    // 对当前新加入的平面点点云中的所有点，进行坐标转换至世界坐标系下，找到其对应的点云栅格坐标
    // 插入到该栅格中存放的点云中
    pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);

    int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
    int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
    int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

    if (pointSel.x + 25.0 < 0)
        cubeI--;
    if (pointSel.y + 25.0 < 0)
        cubeJ--;
    if (pointSel.z + 25.0 < 0)
        cubeK--;

    if (cubeI >= 0 && cubeI < laserCloudWidth &&
        cubeJ >= 0 && cubeJ < laserCloudHeight &&
        cubeK >= 0 && cubeK < laserCloudDepth)
    {
        int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
        laserCloudSurfArray[cubeInd]->push_back(pointSel);
    }
}
printf("add points time %f ms\n", t_add.toc());


TicToc t_filter;
for (int i = 0; i < laserCloudValidNum; i++)
{
    // 对这次处理涉及到的点云进行降采样处理
    int ind = laserCloudValidInd[i];

    pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());
    downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
    downSizeFilterCorner.filter(*tmpCorner);
    laserCloudCornerArray[ind] = tmpCorner;

    pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());
    downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
    downSizeFilterSurf.filter(*tmpSurf);
    laserCloudSurfArray[ind] = tmpSurf;
}
printf("filter time %f ms \n", t_filter.toc());
```

## 全局/局部地图以及位姿发布

按照一定频率发布全局和局部点云地图，以及发布当前估计的（低频）位姿

```c++
if (frameCount % 20 == 0)
{
    // 每 20 帧发布（更新）一次全局地图（将所有点云相加并发布）
    pcl::PointCloud<PointType> laserCloudMap;
    for (int i = 0; i < 4851; i++)
    {
        laserCloudMap += *laserCloudCornerArray[i];
        laserCloudMap += *laserCloudSurfArray[i];
    }
    sensor_msgs::PointCloud2 laserCloudMsg;
    pcl::toROSMsg(laserCloudMap, laserCloudMsg);
    laserCloudMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
    laserCloudMsg.header.frame_id = "camera_init";
    pubLaserCloudMap.publish(laserCloudMsg);
}

// 将当前帧点云（所有点，不限于角点和平面点）转换至世界坐标系下发布
int laserCloudFullResNum = laserCloudFullRes->points.size();
for (int i = 0; i < laserCloudFullResNum; i++)
{
    pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
}

sensor_msgs::PointCloud2 laserCloudFullRes3;
pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
laserCloudFullRes3.header.frame_id = "camera_init";
pubLaserCloudFullRes.publish(laserCloudFullRes3);

printf("mapping pub time %f ms \n", t_pub.toc());

printf("whole mapping time %f ms +++++\n", t_whole.toc());

// 发布更新后的全局位姿和路径（在后端建立的全局坐标系下）
nav_msgs::Odometry odomAftMapped;
odomAftMapped.header.frame_id = "camera_init";
odomAftMapped.child_frame_id = "aft_mapped";
odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
odomAftMapped.pose.pose.position.x = t_w_curr.x();
odomAftMapped.pose.pose.position.y = t_w_curr.y();
odomAftMapped.pose.pose.position.z = t_w_curr.z();
pubOdomAftMapped.publish(odomAftMapped);

geometry_msgs::PoseStamped laserAfterMappedPose;
laserAfterMappedPose.header = odomAftMapped.header;
laserAfterMappedPose.pose = odomAftMapped.pose.pose;
laserAfterMappedPath.header.stamp = odomAftMapped.header.stamp;
laserAfterMappedPath.header.frame_id = "camera_init";
laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
pubLaserAfterMappedPath.publish(laserAfterMappedPath);

// 设置初始位姿（起始点）到当前位姿的转换关系，并发布
static tf::TransformBroadcaster br;
tf::Transform transform;
tf::Quaternion q;
transform.setOrigin(tf::Vector3(t_w_curr(0),
                                t_w_curr(1),
                                t_w_curr(2)));
q.setW(q_w_curr.w());
q.setX(q_w_curr.x());
q.setY(q_w_curr.y());
q.setZ(q_w_curr.z());
transform.setRotation(q);
br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "/camera_init", "/aft_mapped"));

frameCount++;
}
```