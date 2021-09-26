# A-LOAM - kitti helper 代码分析

## 简介

作为对已注释代码的补充说明，主要是进行逻辑上的梳理。

## 整体思路

kitti helper 的作用主要是读取 kitti 的数据并转化为 ros 消息发布或写入 rosbag 中，方便后续节点直接使用。代码整体思路比较简单，整理如下：

- 初始化节点并获取各项参数

参数读取的方式是直接通过 ros 的参数读取接口进行读取。

```c++
ros::init(argc, argv, "kitti_helper");
ros::NodeHandle n("~");
std::string dataset_folder, sequence_number, output_bag_file;
n.getParam("dataset_folder", dataset_folder);
n.getParam("sequence_number", sequence_number);
std::cout << "Reading sequence " << sequence_number 
            << " from " << dataset_folder << '\n';
bool to_bag;
n.getParam("to_bag", to_bag);

if (to_bag)
    n.getParam("output_bag_file", output_bag_file);
int publish_delay;
n.getParam("publish_delay", publish_delay);
publish_delay = publish_delay <= 0 ? 1 : publish_delay;
```

- 初始化 publisher

kitti helper 里分别可以读取 kitti odometry 数据集中的左右灰度图像、位姿参考真值，激光点云以及时间戳作为各个消息的共同参考。节点里初始化了 5 个 publisher，分别发布左右相机图像、激光点云、某一时间戳下的真值位姿以及累计路径。这里有一个小细节注意一下，由于 kitti 的数据集中，位姿真值是通过 gps/imu 的输出经校正后投影到左相机坐标系下的，loam 中使用的是激光点云，所以这里后续会将真值 pose 所在的 camera 系经过旋转和 lidar 系对齐（但还有一定平移）。因此，这里 odom 和 path publisher 的 frame id 是 kitti 原来的 left camera 系经过旋转的。

```c++
// 初始化点云 publisher
ros::Publisher pub_laser_cloud = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2);

// 初始化图像 publisher
image_transport::ImageTransport it(n);
image_transport::Publisher pub_image_left = it.advertise("/image_left", 2);
image_transport::Publisher pub_image_right = it.advertise("/image_right", 2);

// 初始化 odom publisher, 这里的 odom 和 path 的坐标系是 kitti 中原始的 camera 经过旋转的
// 具体见下面数据处理过程
ros::Publisher pubOdomGT = n.advertise<nav_msgs::Odometry> ("/odometry_gt", 5);
nav_msgs::Odometry odomGT;
odomGT.header.frame_id = "/camera_init";
odomGT.child_frame_id = "/ground_truth";

// 初始化路径 publisher
ros::Publisher pubPathGT = n.advertise<nav_msgs::Path> ("/path_gt", 5);
nav_msgs::Path pathGT;
pathGT.header.frame_id = "/camera_init";
```

- 打开时间戳、真值文件，以及要写入的 rosbag

```c++
// 读入时间戳文件
std::string timestamp_path = "sequences/" + sequence_number + "/times.txt";
std::ifstream timestamp_file(dataset_folder + timestamp_path, std::ifstream::in);

// 读入 ground truth 文件
std::string ground_truth_path = "results/" + sequence_number + ".txt";
std::ifstream ground_truth_file(dataset_folder + ground_truth_path, std::ifstream::in);

// 写入模式打开 rosbag （如果非空会覆盖内容）
rosbag::Bag bag_out;
if (to_bag)
    bag_out.open(output_bag_file, rosbag::bagmode::Write);

```

- 设置 camera 旋转矩阵及其余参数

如上所述，这里初始化了一个旋转矩阵用于将 camera 系朝与 lidar 系一致。

```c++
// 旋转 camera 坐标系以对齐 lidar frame
Eigen::Matrix3d R_transform;
R_transform << 0, 0, 1, -1, 0, 0, 0, -1, 0;
Eigen::Quaterniond q_transform(R_transform);

std::string line;
std::size_t line_num = 0;

// 消息发布频率
ros::Rate r(10.0 / publish_delay);
```

接下来的 while 循环中每次从时间戳文件中读取一个文件，然后进行以下操作：

- 读入图像，分别读入一对左右相机图像，这里原代码使用的 `CV_LOAD_IMAGE_GRAYSCALE` 宏定义在 opencv3.1 之后已经被取消了，读取灰度图像应该用 `cv::IMREAD_GRAYSCALE`

```c++
cv::Mat left_image = cv::imread(left_image_path.str(), CV_LOAD_IMAGE_GRAYSCALE);
// 读入图像
float timestamp = stof(line);
std::stringstream left_image_path, right_image_path;
left_image_path << dataset_folder << "sequences/" + sequence_number + "/image_0/" << std::setfill('0') << std::setw(6) << line_num << ".png";
cv::Mat left_image = cv::imread(left_image_path.str(), cv::IMREAD_GRAYSCALE);
// out-of-date after opencv 3.1
//cv::Mat left_image = cv::imread(left_image_path.str(), CV_LOAD_IMAGE_GRAYSCALE);
right_image_path << dataset_folder << "sequences/" + sequence_number + "/image_1/" << std::setfill('0') << std::setw(6) << line_num << ".png";
cv::Mat right_image = cv::imread(left_image_path.str(), cv::IMREAD_GRAYSCALE);
// cv::Mat right_image = cv::imread(right_image_path.str(), CV_LOAD_IMAGE_GRAYSCALE);
```

- 读入 3x4 真值 pose

先从文件中读取一行存入字符串中，每行真值数据包括 12 个 float 数据，再从字符串流中以空格为分隔符按行优先的顺序读入 3x4 位姿矩阵中。

```c++
// 真值 pose 按行读入 3x4 位姿矩阵
std::getline(ground_truth_file, line);
std::stringstream pose_stream(line);
std::string s;
Eigen::Matrix<double, 3, 4> gt_pose;
for (std::size_t i = 0; i < 3; ++i)
{
    for (std::size_t j = 0; j < 4; ++j)
    {
        std::getline(pose_stream, s, ' ');
        gt_pose(i, j) = stof(s);
    }
}
```

- 将真值更新到要发布的 pose 和 path中

前面提到这里作者为了统一坐标系的方向，因此这里对相机坐标系进行旋转使之与 lidar 坐标系方向一致，将其作为里程计真值发布，并更新到路径消息中。注意这里作者使用的四元数作为旋转方法，而四元数旋转向量的方法应该是：`p' = qpq^-1`,所以这里有一个小 bug。

```c++
// 将 camera 坐标系下的 groungtruth pose 转到 lidar 坐标系下作为 groundtruth
Eigen::Quaterniond q_w_i(gt_pose.topLeftCorner<3, 3>());

// 四元数的坐标转换应该是 p' = qpq^-1
// Eigen::Quaterniond q = q_transform * q_w_i;
Eigen::Quaterniond q = q_transform * q_w_i * q_transform.inverse();
q.normalize();
Eigen::Vector3d t = q_transform * gt_pose.topRightCorner<3, 1>();

// 发布里程计 ground truth
odomGT.header.stamp = ros::Time().fromSec(timestamp);
odomGT.pose.pose.orientation.x = q.x();
odomGT.pose.pose.orientation.y = q.y();
odomGT.pose.pose.orientation.z = q.z();
odomGT.pose.pose.orientation.w = q.w();
odomGT.pose.pose.position.x = t(0);
odomGT.pose.pose.position.y = t(1);
odomGT.pose.pose.position.z = t(2);
pubOdomGT.publish(odomGT);

// 发布路径 ground truth
geometry_msgs::PoseStamped poseGT;
poseGT.header = odomGT.header;
poseGT.pose = odomGT.pose.pose;

pathGT.header.stamp = odomGT.header.stamp;
pathGT.poses.push_back(poseGT);
pubPathGT.publish(pathGT);
```

- 将 lidar 数据读取至 float 数据中

kitti 中的 lidar 数据是存在一个二进制文件里，没一个 lidar 数据包括四个参数：x, y, z, intensity，各个数据紧凑排列，因此这里作者使用一个函数先从文件中将数据读取到 float 数组中方便后续操作。读取过程比较简单粗暴，直接统计数据长度，将该长度的 byte 读入数组中，然后返回数组。

```c++
// 将 lidar 数据按 float 读入 vector 中
// 单个 lidar 数据格式为： x，y，z，intensity
std::stringstream lidar_data_path;
lidar_data_path << dataset_folder << "velodyne/sequences/" + sequence_number + "/velodyne/" 
                << std::setfill('0') << std::setw(6) << line_num << ".bin";
std::vector<float> lidar_data = read_lidar_data(lidar_data_path.str());
std::cout << "totally " << lidar_data.size() / 4.0 << " points in this lidar frame \n";

///
///@brief 从文件中读取 lidar 数据至 float 数据
///
///@param lidar_data_path 
///@return std::vector<float> 
///
std::vector<float> read_lidar_data(const std::string lidar_data_path)
{
    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    
    // 将下一字符定位到文件末尾
    lidar_data_file.seekg(0, std::ios::end);
    // 获取文件 byte 数量进而求 float 的数量
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    // 将下一字符复位至文件起始位置
    lidar_data_file.seekg(0, std::ios::beg);

    // 从文件中读取所有 byte 至数组中
    std::vector<float> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements*sizeof(float));
    return lidar_data_buffer;
}
```

- 将 lidar 数据读入点云中，这里作者还有两个数组分别存储各个点的坐标和intensity 但其实没有用到。

```c++
// 按 x，y，z，intensity 的顺序读取每个lidar point 数据并放入 pcl 点云中

// std::vector<Eigen::Vector3d> lidar_points;
// std::vector<float> lidar_intensities;
pcl::PointCloud<pcl::PointXYZI> laser_cloud;
for (std::size_t i = 0; i < lidar_data.size(); i += 4)
{
    // lidar_points.emplace_back(lidar_data[i], lidar_data[i+1], lidar_data[i+2]);
    // lidar_intensities.push_back(lidar_data[i+3]);

    pcl::PointXYZI point;
    point.x = lidar_data[i];
    point.y = lidar_data[i + 1];
    point.z = lidar_data[i + 2];
    point.intensity = lidar_data[i + 3];
    laser_cloud.push_back(point);
}
```

- 图像、点云消息发布以及写入 rosbag 中（可选）

```c++
// 发布点云
sensor_msgs::PointCloud2 laser_cloud_msg;
pcl::toROSMsg(laser_cloud, laser_cloud_msg);
laser_cloud_msg.header.stamp = ros::Time().fromSec(timestamp);
laser_cloud_msg.header.frame_id = "/camera_init";
pub_laser_cloud.publish(laser_cloud_msg);

// 发布左右图像
sensor_msgs::ImagePtr image_left_msg = cv_bridge::CvImage(laser_cloud_msg.header, "mono8", left_image).toImageMsg();
sensor_msgs::ImagePtr image_right_msg = cv_bridge::CvImage(laser_cloud_msg.header, "mono8", right_image).toImageMsg();
pub_image_left.publish(image_left_msg);
pub_image_right.publish(image_right_msg);

if (to_bag)
{
    // 写入 rosbag 中，注意此时groundtruth 的坐标系为 lidar
    bag_out.write("/image_left", ros::Time::now(), image_left_msg);
    bag_out.write("/image_right", ros::Time::now(), image_right_msg);
    bag_out.write("/velodyne_points", ros::Time::now(), laser_cloud_msg);
    bag_out.write("/path_gt", ros::Time::now(), pathGT);
    bag_out.write("/odometry_gt", ros::Time::now(), odomGT);
}
```