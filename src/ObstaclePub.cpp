#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>


#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/filters/voxel_grid.h>



class PointCloudFilter {
public:
    PointCloudFilter(ros::NodeHandle& nh)
    {
        // Subscribers and publishers
        cloud_sub_ = nh.subscribe("/groundgrid/segmented_cloud", 1, &PointCloudFilter::cloudCallback, this);
        odom_sub_ = nh.subscribe("/Odometry", 1, &PointCloudFilter::odomCallback, this);
        have_odom = false;
 
        obstacle_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/groundgrid/obstacle_cloud", 1);
        ground_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/groundgrid/ground_cloud", 1);
        line_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/groundgrid/line_cloud", 1);

    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
    {
        have_odom = true;
        pose_ = odom_msg->pose;
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*input, *cloud);

        // Create a new filtered cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr line_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

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

        pcl::PointCloud<pcl::PointXYZI>::Ptr away_from_origin(new pcl::PointCloud<pcl::PointXYZI>);

        // remove points immediately near robot. too dense for line detection
        if (have_odom) 
            filter_points_near_origin(ground_cloud_filtered, away_from_origin, 9);
        else
            away_from_origin.swap(ground_cloud_filtered);

        ROS_WARN("input size %d output size %d (have odom)", ground_cloud_filtered->size(), away_from_origin->size(), have_odom);

        // line_cloud_filtered = lineSegmentation(away_from_origin);
        // publish points
        // sensor_msgs::PointCloud2 line_output;
        // pcl::toROSMsg(*line_cloud_filtered, line_output);
        // line_output.header = input->header;
        // line_pub_.publish(line_output);
    }


    void filter_points_near_origin(pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud, 
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr& output_cloud, 
                                    float distance_threshold)
    {
        output_cloud->clear();

        for (const auto& point : input_cloud->points)
        {
            float diff_x = pose_.pose.position.x - point.x;
            float diff_y = pose_.pose.position.y - point.y;
            float diff_z = pose_.pose.position.z - point.z;

            float distance_to_origin = std::sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
            if (distance_to_origin < distance_threshold)
            {
                output_cloud->points.push_back(point);
            }
        }

        output_cloud->width = output_cloud->points.size();
        output_cloud->height = 1; // Unorganized point cloud
        output_cloud->is_dense = true;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr lineSegmentation(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
    {
        // Downsample the point cloud
        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
        voxel_grid.setInputCloud(cloud);
        voxel_grid.setLeafSize(0.05f, 0.05f, 0.05f);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZI>);
        voxel_grid.filter(*cloud_downsampled);

        // // Detect line segments using RANSAC
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_LINE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.025);
        seg.setMaxIterations(50);
        seg.setInputCloud(cloud_downsampled);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_lines(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::ExtractIndices<pcl::PointXYZI> extract;


        int n_iters = 0;
        int total_lines_detected = 0;
        while (cloud_downsampled->points.size() > 10 && n_iters++ < 100)
        {

            seg.setInputCloud(cloud_downsampled);
            seg.segment(*inliers, *coefficients);
            if (inliers->indices.size() == 0)
                break;

            // Extract the inliers (line points)
            extract.setInputCloud(cloud_downsampled);
            extract.setIndices(inliers);
            extract.setNegative(false);
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZI>);
            extract.filter(*cloud_line);

            if (cloud_line->points.size() > 100)
                *cloud_lines += *cloud_line;

            // Print the number of points fitting the detected line
            ROS_INFO("Line %d detected with %lu points fitting it (%d size cloud)", total_lines_detected + 1, cloud_line->points.size(), cloud_downsampled->size());

            ROS_INFO("coef: (%d, %d, %d, %d, %d, %d)", coefficients->values[0], coefficients->values[1],coefficients->values[2],coefficients->values[3],coefficients->values[4],coefficients->values[5]);
            total_lines_detected++;

            // Remove the inliers, extract the rest
            extract.setNegative(true);
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_rest(new pcl::PointCloud<pcl::PointXYZI>);
            extract.filter(*cloud_rest);
            cloud_downsampled = cloud_rest;

            // // Segment the largest line component
            // seg.segment(*inliers, *coefficients);
            // if (inliers->indices.size() == 0)
            //     break;

            // ROS_WARN("Found %d inliers have %d points ", inliers->indices.size(), cloud_downsampled->size());
            // ROS_WARN("have coefficients %f, %f, %f", coefficients->values[0], coefficients->values[1], coefficients->values[2]);

            // // Extract the inliers (line points)
            // extract.setInputCloud(cloud_downsampled);
            // extract.setIndices(inliers);
            // extract.setNegative(false);
            // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZI>);
            // extract.filter(*cloud_line);
            // *cloud_lines += *cloud_line;

            // // Remove the inliers, extract the rest
            // extract.setNegative(true);
            // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_rest(new pcl::PointCloud<pcl::PointXYZI>);
            // extract.filter(*cloud_rest);
            // cloud_downsampled.swap(cloud_rest);
        }

        return cloud_lines;
    }

private:
    ros::Subscriber cloud_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher obstacle_pub_;
    ros::Publisher ground_pub_;
    ros::Publisher line_pub_;

    geometry_msgs::PoseWithCovariance pose_;
    bool have_odom;


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