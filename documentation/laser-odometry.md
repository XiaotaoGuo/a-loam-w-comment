# A-LOAM - laser odometry 代码分析

## 简介

作为对已注释代码的补充说明，主要是进行逻辑上的梳理。

## 整体思路

laserOdometry 节点的主要作用是作为 a loam 的里程计对激光雷达的位姿进行快速的估计，大致过程为：

- 从 scanRegistraion 中接收预处理后的点云以及各个特征点点云，并将其存放在相应的缓冲队列中
- 每一次从各个缓冲队列中获取最早的一帧点云以及相应的特征点
- 对每一个特征点（平面点/角点），在上一帧中找到相应的特征点及其构成的点，对角点来说是两个点构成一个角点特征；对平面点来说是三个点构成一个平面
- 将当前分析的特征点以及找到的匹配特征点作为观测值加入 ceres 问题中构建残差
- 对所有观测值进行优化
- 更新世界坐标系下的位姿以及发布更新后的点云地图

## 接收点云并存放

主要就是一系列 ROS subscribers 及其回调函数的作用，比较简单。

## Ceres 初始化

代码中进行两次优化，每一次单独设置 ceres，如下所示，关于 ceres 的基本使用方法可以参考[Ceres 学习记录（一）](https://xiaotaoguo.com/p/ceres-usage/)，这里将待优化的旋转平移作为参数块添加进问题中：

```c++
//ceres::LossFunction *loss_function = NULL;
ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
ceres::LocalParameterization *q_parameterization =
    new ceres::EigenQuaternionParameterization();
ceres::Problem::Options problem_options;

ceres::Problem problem(problem_options);
problem.AddParameterBlock(para_q, 4, q_parameterization);
problem.AddParameterBlock(para_t, 3);
```

## 对当前角点在上一帧寻找匹配角点

对每一个角点在上一帧点云中寻找角点的逻辑和论文中基本类似，整体流程如下：

- 将当前点 p 转换到上一帧点云的时间上
- 通过 KD 树在上一帧点云上上找到最近点 a
- 在 a 所属的扫描线的相邻扫描线（代码中是上下相邻 2 条线）中寻找另一个离 p 最近的点 b
- 计算畸变因子（和该帧点云的起始点的相对时间差）
- 将点 a, b, p 以及畸变因子构建 loss 函数跟旋转和平移参数构建残差块传入 ceres 问题中
  - 计算残差的方法和论文基本一致：
  - 按需对点 p 进行去畸变处理，这里去畸变的做法时假定在一帧点云中激光雷达做匀速运动，因此对旋转和平移进行线性插值即可，对于更多的去畸变方法可以参考：[激光雷达测量模型及运动畸变去除](https://xiaotaoguo.com/p/lidar-model-distortion-removal/)，如果使用的是类似 Kitti 数据集之类的已经校正过的点云则不要去畸变
  - 计算 pa 和 pb 的叉乘，表示三角形 abp 的面积，然后除以 ab 即可得到距离

所有特征点的残差块加入后优化即可。

## 对当前平面点在上一帧寻找匹配平面点

过程和角点匹配类似：

- 将当前点 p 转换到上一帧点云的时间上
- 通过 KD 树在上一帧点云上上找到最近点 l
- 在 l 点上面或者同一扫描线上找一最近点 j
- 在 l 点下面的扫描线找一最近点 m
- 同样思路按需进行畸变因子的计算，和 p, l, j, m 构建损失函数，将损失函数和旋转平移参数一起构建残差块加入 ceres 问题中
  - 残差方式和论文一致：
  - 通过 lj, lm 叉乘归一化求得平面 ljm 的单位法向量 n
  - p 和平面上任意一点连线和 n 点乘即可得到 p 到平面 ljm 的距离

所有特征点的残差块加入后优化即可。

## 更新路径信息和当前对世界的位姿

更新世界坐标系下的坐标为，注意这里四元数和向量可以直接相乘是因为 `eigen` 重载了四元数的乘法，如果自己实现的时候要注意四元数乘法和普通矩阵乘法的区别：

```c++
t_w_curr = t_w_curr + q_w_curr * t_last_curr;
q_w_curr = q_w_curr * q_last_curr;
``

这里有一点不太清楚的地方是作者将 `TransformToEnd` 这一步将当前帧点云转换到结束时刻的位姿的部分注释掉了，即默认是不进行这一操作的，但是论文里应该是有这一操作的，因为在我们对特征点匹配的时候，我们是将特征点转换到当前帧起始时间，即上一帧的结束时间的。能想到的一个解释是对于 Kitti 等数据集中，其发布的点云经过校正后本来就是对应当前帧结束时刻的位姿，但是需要看数据集确定。

```c++
// transform corner features and plane features to the scan end point
// why don't run this process?
if (0)
{
    int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
    for (int i = 0; i < cornerPointsLessSharpNum; i++)
    {
        TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
    }

    int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
    for (int i = 0; i < surfPointsLessFlatNum; i++)
    {
        TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
    }

    int laserCloudFullResNum = laserCloudFullRes->points.size();
    for (int i = 0; i < laserCloudFullResNum; i++)
    {
        TransformToEnd(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
    }
}
```

最后按照给定的频率发布当前的累计点云地图：

```c++
// 按给定频率发布当前的点云地图
if (frameCount % skipFrameNum == 0)
{
    frameCount = 0;

    sensor_msgs::PointCloud2 laserCloudCornerLast2;
    pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
    laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
    laserCloudCornerLast2.header.frame_id = "camera";
    pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

    sensor_msgs::PointCloud2 laserCloudSurfLast2;
    pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
    laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
    laserCloudSurfLast2.header.frame_id = "camera";
    pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
    laserCloudFullRes3.header.frame_id = "camera";
    pubLaserCloudFullRes.publish(laserCloudFullRes3);
}
```