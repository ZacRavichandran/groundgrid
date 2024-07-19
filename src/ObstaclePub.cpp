#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>

class PointCloudFilter {
public:
    PointCloudFilter(ros::NodeHandle& nh)
    {
        // Subscribers and publishers
        cloud_sub_ = nh.subscribe("/groundgrid/segmented_cloud", 1, &PointCloudFilter::cloudCallback, this);
        obstacle_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/groundgrid/obstacle_cloud", 1);
        ground_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/groundgrid/ground_cloud", 1);
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*input, *cloud);

        // Create a new filtered cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

        for (auto& point : cloud->points) {
            // Check if the point is approximately purple
            if (isObstacle(point)) {
                obstacle_cloud_filtered->points.push_back(point);
            } else {
                ground_cloud_filtered->points.push_back(point);
            }
        }

        // Convert to ROS data type and publish
        sensor_msgs::PointCloud2 obstacle_output;
        pcl::toROSMsg(*obstacle_cloud_filtered, obstacle_output);
        obstacle_output.header = input->header;
        obstacle_pub_.publish(obstacle_output);

        // now publish ground
        sensor_msgs::PointCloud2 ground_output;
        pcl::toROSMsg(*ground_cloud_filtered, ground_output);
        ground_output.header = input->header;
        ground_pub_.publish(ground_output);
    }

private:
    ros::Subscriber cloud_sub_;
    ros::Publisher obstacle_pub_;
    ros::Publisher ground_pub_;


    bool isObstacle(const pcl::PointXYZI& point)
    {
        // Define color tolerance for purple (you can fine-tune these values)
        // purpose is 255, 255, 60
        int r_min = 250, r_max = 260;
        int g_min = 40, g_max = 60;
        int b_min = 250, b_max = 260;

        return point.intensity == 99;


        // std::cout << point.r << ", " << point.g << ", " << point.b << std::endl;
        // return (point.r >= r_min && point.r <= r_max &&
        //         point.g >= g_min && point.g <= g_max &&
        //         point.b >= b_min && point.b <= b_max);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_pub");
    ros::NodeHandle nh;
    PointCloudFilter filter(nh);
    ros::spin();
    return 0;
}