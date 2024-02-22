#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <iostream>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "maps.hpp"


mocka::Maps::BasicInfo info;

class MockaMapNode : public rclcpp::Node
{
public:
  MockaMapNode()
    : Node("mockamap")
  {
    // Initialize parameters
    this->declare_parameter("numNodes", 10);
    this->declare_parameter("connectivity", 0.5);
    this->declare_parameter("nodeRad", 3);
    this->declare_parameter("roadRad", 2);
    this->declare_parameter("width_min", 0.6);
    this->declare_parameter("width_max", 1.5);
    this->declare_parameter("obstacle_number", 10);
    this->declare_parameter("complexity", 0.142857);
    this->declare_parameter("fill", 0.38);
    this->declare_parameter("fractal", 1);
    this->declare_parameter("attenuation", 0.5);
    this->declare_parameter("road_width", 1.0);
    this->declare_parameter("add_wall_x", 0);
    this->declare_parameter("add_wall_y", 0);
    this->declare_parameter("maze_type", 1);

    this->declare_parameter<int>("seed", 4546);
    this->declare_parameter<double>("update_freq", 1.0);
    this->declare_parameter<double>("resolution", 0.38);
    this->declare_parameter<int>("x_length", 100);
    this->declare_parameter<int>("y_length", 100);
    this->declare_parameter<int>("z_length", 10);
    this->declare_parameter<int>("type", 1);

    // Retrieve parameters
    this->get_parameter("seed", seed);
    this->get_parameter("update_freq", update_freq);
    this->get_parameter("resolution", scale);
    this->get_parameter("x_length", sizeX);
    this->get_parameter("y_length", sizeY);
    this->get_parameter("z_length", sizeZ);
    this->get_parameter("type", type);

    scale = 1 / scale;
    sizeX = sizeX * scale;
    sizeY = sizeY * scale;
    sizeZ = sizeZ * scale;

    info.sizeX  = sizeX;
    info.sizeY  = sizeY;
    info.sizeZ  = sizeZ;
    info.seed   = seed;
    info.scale  = scale;
    info.output = &output;
    info.cloud  = &cloud;

    pcl_pub =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("mock_map", 1);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(update_freq * 1000)),
      std::bind(&MockaMapNode::publishPointCloud, this));
  }

private:
  // void asdf(rclcpp::Node* n){
  //     int seed;
  //     n->get_parameter("seed", seed);
  //     RCLCPP_INFO_STREAM(n->get_logger(), "seed" << seed << "\n") ;
  // }
  void publishPointCloud()
  {
    pcl_pub->publish(output);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub;
  rclcpp::TimerBase::SharedPtr                                timer_;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::msg::PointCloud2  output;

  int    seed;
  double scale;
  double update_freq;
  int    sizeX, sizeY, sizeZ;
  int    type;
};

void
optimizeMap(mocka::Maps::BasicInfo& in)
{
  std::vector<int>* temp = new std::vector<int>;

  pcl::KdTreeFLANN<pcl::PointXYZ>     kdtree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  cloud->width  = in.cloud->width;
  cloud->height = in.cloud->height;
  cloud->points.resize(cloud->width * cloud->height);

  for (int i = 0; i < cloud->width; i++)
  {
    cloud->points[i].x = in.cloud->points[i].x;
    cloud->points[i].y = in.cloud->points[i].y;
    cloud->points[i].z = in.cloud->points[i].z;
  }

  kdtree.setInputCloud(cloud);
  double radius = 1.75 / in.scale; // 1.75 is the rounded up value of sqrt(3)

  for (int i = 0; i < cloud->width; i++)
  {
    std::vector<int>   pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    if (kdtree.radiusSearch(cloud->points[i],
                            radius,
                            pointIdxRadiusSearch,
                            pointRadiusSquaredDistance) >= 27)
    {
      temp->push_back(i);
    }
  }
  for (int i = temp->size() - 1; i >= 0; i--)
  {
    in.cloud->points.erase(in.cloud->points.begin() +
                           temp->at(i)); // erasing the enclosed points
  }
  in.cloud->width -= temp->size();

  pcl::toROSMsg(*in.cloud, *in.output);
  in.output->header.frame_id = "odom";
  printf("finish: number of points after optimization %d", in.cloud->width);
  delete temp;
  return;
}

int
main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MockaMapNode>();
  mocka::Maps map(node);
  map.setInfo(info);
  int type = 1;
  node->get_parameter("type", type);
  map.generate(type);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
