#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "kml/base/file.h"
#include "kml/base/version.h"
#include "kml/dom.h"
#include "kml/engine.h"
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>


using namespace std::chrono_literals;

class KmlParserNode:public rclcpp::Node{
    public:
        KmlParserNode();
    private:

        void read_coordinates_from_file(std::string filename);
        void project_coordinates();
        void publish_path();

        kmlengine::KmlFilePtr kml_file_ptr_;
        kmldom::CoordinatesPtr coordinates_ptr;

        nav_msgs::msg::Path path_msg;

        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
};

KmlParserNode::KmlParserNode():Node("kml_parser_node")
{
  std::string file_name = this->declare_parameter("kml_file_path", "src/kml_parser/data/test_route.kml");

  this->path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  read_coordinates_from_file(file_name);
  project_coordinates();
  timer_ = this->create_wall_timer(500ms, std::bind(&KmlParserNode::publish_path, this));
}

void KmlParserNode::read_coordinates_from_file(std::string filename)
{
  // read kml file content
  std::string kml_file_content;

  if(!kmlbase::File::ReadFileToString(filename, &kml_file_content))
  {
    std::cout << "File Read failed with path: " << filename << std::endl;
    return;
  }

  kml_file_ptr_ = kmlengine::KmlFile::CreateFromString(kml_file_content);

  kmldom::PlacemarkPtr placemark = AsPlacemark(kml_file_ptr_->GetObjectById("0C11468A89207A6E17BE"));
  coordinates_ptr = AsLineString(placemark->get_geometry())->get_coordinates();
}

void KmlParserNode::project_coordinates()
{
  double x, y, z;
  geometry_msgs::msg::PoseStamped pose_msg;
  GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
  GeographicLib::LocalCartesian projector(coordinates_ptr->get_coordinates_array_at(0).get_latitude(),
   coordinates_ptr->get_coordinates_array_at(0).get_longitude(), coordinates_ptr->get_coordinates_array_at(0).get_altitude(), earth);

  for (size_t i = 1; i < coordinates_ptr->get_coordinates_array_size(); i++)
  {
    projector.Forward(coordinates_ptr->get_coordinates_array_at(i).get_latitude(), coordinates_ptr->get_coordinates_array_at(i).get_longitude(),
                        coordinates_ptr->get_coordinates_array_at(0).get_altitude(), x, y, z);

    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.position.z = z;

    path_msg.poses.push_back(pose_msg);
  }
}

void KmlParserNode::publish_path()
{
  path_msg.header.stamp = this->get_clock()->now();
  path_msg.header.frame_id = "map";

  path_publisher_->publish(path_msg);
}

int main(int argc, char* argv[]){    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KmlParserNode>());
    rclcpp::shutdown();
    return 0;
}