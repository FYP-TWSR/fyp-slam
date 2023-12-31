#include "orb_slam3.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    rgb_sub = std::make_shared< message_filters::Subscriber<ImageMsg> >(this, "/camera/color/image_raw");
    depth_sub = std::make_shared< message_filters::Subscriber<ImageMsg> >(this, "/camera/aligned_depth_to_color/image_raw");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBD, this);

    pcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("realsense_pts", 1);
    pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("camera_pose", 1);

}

RgbdSlamNode::~RgbdSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    // m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void RgbdSlamNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
{
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    double seconds = msgRGB->header.stamp.sec + (msgRGB->header.stamp.nanosec * pow(10,-9));
    Sophus::SE3f Tcw = m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, seconds);
    Sophus::SE3f Twc = Tcw.inverse();
    rclcpp::Time timestamp = msgRGB->header.stamp;
    sensor_msgs::msg::PointCloud2 cloud = tracked_mappoints_to_pointcloud(m_SLAM->GetTrackedMapPoints());
    geometry_msgs::msg::PoseStamped pose = getPose(Twc);
    pose.header.stamp = timestamp;
    pose_pub->publish(pose);
    cloud.header.stamp = timestamp;
    pcloud_pub->publish(cloud);
}

sensor_msgs::msg::PointCloud2 RgbdSlamNode::tracked_mappoints_to_pointcloud(std::vector<ORB_SLAM3::MapPoint*> map_points)
{
    const int num_channels = 3; // x y z

    if (map_points.size() == 0)
    {
        std::cout << "Map point vector is empty!" << std::endl;
    }

    sensor_msgs::msg::PointCloud2 cloud;

    cloud.header.frame_id = world_frame_id;
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = { "x", "y", "z"};

    for (int i = 0; i < num_channels; i++)
    {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    // unsigned char *cloud_data_ptr = &(cloud.data[0]);


    for (unsigned int i = 0; i < cloud.width; i++)
    {
        if (map_points[i])
        {
            // Original data
            Eigen::Vector3f pMPw = map_points[i]->GetWorldPos();
            
            Eigen::Vector3f point_translation(pMPw.x(), pMPw.y(), pMPw.z());

            float data_array[num_channels] = {
                point_translation.x(),
                point_translation.y(),
                point_translation.z()
            };

            memcpy(&cloud.data+(i*cloud.point_step), data_array, num_channels*sizeof(float));
        }
    }
    return cloud;
}

geometry_msgs::msg::TransformStamped RgbdSlamNode::getTF(Sophus::SE3f se3fTrans)
{
    geometry_msgs::msg::TransformStamped t;

    t.header.frame_id = world_frame_id;
    t.child_frame_id = camera_frame_id;

    t.transform.translation.x = se3fTrans.translation().x();
    t.transform.translation.y = se3fTrans.translation().y();
    t.transform.translation.z = se3fTrans.translation().z();

    t.transform.rotation.x = se3fTrans.unit_quaternion().coeffs().x();
    t.transform.rotation.y = se3fTrans.unit_quaternion().coeffs().y();
    t.transform.rotation.z = se3fTrans.unit_quaternion().coeffs().z();
    t.transform.rotation.w = se3fTrans.unit_quaternion().coeffs().w();
    return t;
}

geometry_msgs::msg::PoseStamped RgbdSlamNode::getPose(Sophus::SE3f se3fTrans)
{
    geometry_msgs::msg::PoseStamped p;

    p.header.frame_id = world_frame_id;

    p.pose.position.x = se3fTrans.translation().x();
    p.pose.position.y = se3fTrans.translation().y();
    p.pose.position.z = se3fTrans.translation().z();

    p.pose.orientation.x = se3fTrans.unit_quaternion().coeffs().x();
    p.pose.orientation.y = se3fTrans.unit_quaternion().coeffs().y();
    p.pose.orientation.z = se3fTrans.unit_quaternion().coeffs().z();
    p.pose.orientation.w = se3fTrans.unit_quaternion().coeffs().w();
    return p;
}

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam rgbd path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    bool visualization = true;
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, visualization);
    auto node = std::make_shared<RgbdSlamNode>(&SLAM);
    std::cout << "============================ " << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}