#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>

class Yaml_parsing : public rclcpp::Node
{
public:
  Yaml_parsing() : Node("yaml_parsing_node")
  {
    this->declare_parameter<std::vector<double>>("footprint.footprint_polygon", std::vector<double>());
    this->declare_parameter<double>("footprint.circular_footprint_radius", 0.0);
    this->declare_parameter<double>("footprint.circular_footprint_radius_inscribed", 0.0);
    this->declare_parameter<double>("footprint.circular_footprint_offset", 0.0);
    this->declare_parameter<std::string>("footprint.footprint_frame_id", "");
    this->declare_parameter<double>("footprint.traversability_default", 0.0);
    this->declare_parameter<bool>("footprint.verify_roughness_footprint", false);
    this->declare_parameter<bool>("footprint.check_robot_inclination", false);

    std::vector<double> footprint_polygon;
    this->get_parameter("footprint.footprint_polygon", footprint_polygon);
    double circular_footprint_radius;
    this->get_parameter("footprint.circular_footprint_radius", circular_footprint_radius);
    double circular_footprint_radius_inscribed;
    this->get_parameter("footprint.circular_footprint_radius_inscribed", circular_footprint_radius_inscribed);
    double circular_footprint_offset;
    this->get_parameter("footprint.circular_footprint_offset", circular_footprint_offset);
    std::string footprint_frame_id;
    this->get_parameter("footprint.footprint_frame_id", footprint_frame_id);
    double traversability_default;
    this->get_parameter("footprint.traversability_default", traversability_default);
    bool verify_roughness_footprint;
    this->get_parameter("footprint.verify_roughness_footprint", verify_roughness_footprint);
    bool check_robot_inclination;
    this->get_parameter("footprint.check_robot_inclination", check_robot_inclination);

    RCLCPP_INFO(this->get_logger(), "Footprint polygon: ");
    for (auto point : footprint_polygon)
    {
      RCLCPP_INFO(this->get_logger(), "%f", point);
    }
    RCLCPP_INFO(this->get_logger(), "Circular footprint radius: %f", circular_footprint_radius);
    RCLCPP_INFO(this->get_logger(), "Circular footprint radius inscribed: %f", circular_footprint_radius_inscribed);
    RCLCPP_INFO(this->get_logger(), "Circular footprint offset: %f", circular_footprint_offset);
    RCLCPP_INFO(this->get_logger(), "Footprint frame ID: %s", footprint_frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "Traversability default: %f", traversability_default);
    RCLCPP_INFO(this->get_logger(), "Verify roughness footprint: %s", verify_roughness_footprint ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Check robot inclination: %s", check_robot_inclination ? "true" : "false");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Yaml_parsing>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
