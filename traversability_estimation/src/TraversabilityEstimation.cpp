/*
 * TraversabilityEstimation.cpp
 *
 *  Created on: Oct 22, 2014
 *      Author: Ralf Kaestner, Peter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 * 
 *  PORTED TO ROS2: Muhammad Ahmed
 */

#include "traversability_estimation/TraversabilityEstimation.hpp"
#include "traversability_estimation/common.h"
#include <traversability_msgs/msg/traversability_result.hpp>

// ROS
#include <geometry_msgs/msg/pose.hpp>
#include <thread>
#include <atomic>

using namespace std;

namespace traversability_estimation {

TraversabilityEstimation::TraversabilityEstimation(rclcpp::Node::SharedPtr& nodeHandle)
    : nodeHandle_(nodeHandle),
      acceptGridMapToInitTraversabilityMap_(false),
      traversabilityMap_(nodeHandle),
      traversabilityType_("traversability"),
      slopeType_("traversability_slope"),
      stepType_("traversability_step"),
      roughnessType_("traversability_roughness"),
      robotSlopeType_("robot_slope"),
      getImageCallback_(false),
      useRawMap_(true),
      tfBuffer_(nodeHandle->get_clock()),
      transformListener_(tfBuffer_),
      updateDuration_(std::chrono::seconds(1)){
  RCLCPP_DEBUG(nodeHandle_->get_logger(), "Traversability estimation node started.");

  readParameters();
  traversabilityMap_.createLayers(useRawMap_);
  submapClient_ = nodeHandle_->create_client<grid_map_msgs::srv::GetGridMap>(submapServiceName_);

  if (updateDuration_.nanoseconds() != 0) {
    RCLCPP_INFO(nodeHandle_->get_logger(), "Update rate is : %f Hz.", 1.0 / updateDuration_.seconds());
    updateTimer_ = nodeHandle_->create_wall_timer(
      std::chrono::nanoseconds(updateDuration_.nanoseconds()),
      std::bind(&TraversabilityEstimation::updateTimerCallback, this)
    );
  } else {
    RCLCPP_WARN(nodeHandle_->get_logger(), "Update rate is zero. No traversability map will be published.");
  }

  loadElevationMapService_ = nodeHandle_->create_service<grid_map_msgs::srv::ProcessFile>(
    "load_elevation_map", std::bind(&TraversabilityEstimation::loadElevationMap, this, std::placeholders::_1, std::placeholders::_2));

  updateTraversabilityService_ = nodeHandle_->create_service<grid_map_msgs::srv::GetGridMapInfo>(
    "update_traversability", std::bind(&TraversabilityEstimation::updateServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
  
  getTraversabilityService_ = nodeHandle_->create_service<grid_map_msgs::srv::GetGridMap>(
    "get_traversability", std::bind(&TraversabilityEstimation::getTraversabilityMap, this, std::placeholders::_1, std::placeholders::_2));
  
  footprintPathService_ = nodeHandle_->create_service<traversability_msgs::srv::CheckFootprintPath>(
    "check_footprint_path", std::bind(&TraversabilityEstimation::checkFootprintPath, this, std::placeholders::_1, std::placeholders::_2));
    
  updateParameters_ = nodeHandle_->create_service<std_srvs::srv::Empty>(
    "update_parameters", std::bind(&TraversabilityEstimation::updateParameter, this, std::placeholders::_1, std::placeholders::_2));
    
  traversabilityFootprint_ = nodeHandle_->create_service<std_srvs::srv::Empty>(
      "traversability_footprint", std::bind(&TraversabilityEstimation::traversabilityFootprint, this, std::placeholders::_1, std::placeholders::_2));

  saveToBagService_ = nodeHandle_->create_service<grid_map_msgs::srv::ProcessFile>(
      "save_traversability_map_to_bag", std::bind(&TraversabilityEstimation::saveToBag, this, std::placeholders::_1, std::placeholders::_2));

  imageSubscriber_ = nodeHandle_->create_subscription<sensor_msgs::msg::Image>(
      imageTopic_, 1, std::bind(&TraversabilityEstimation::imageCallback, this, std::placeholders::_1));

  if (acceptGridMapToInitTraversabilityMap_) {
    gridMapToInitTraversabilityMapSubscriber_ = nodeHandle_->create_subscription<grid_map_msgs::msg::GridMap>(
        gridMapToInitTraversabilityMapTopic_, 1, std::bind(&TraversabilityEstimation::gridMapToInitTraversabilityMapCallback, this, std::placeholders::_1));
  }

  elevationMapLayers_.push_back("elevation");
  if (!useRawMap_) {
    elevationMapLayers_.push_back("upper_bound");
    elevationMapLayers_.push_back("lower_bound");
  } else {
    elevationMapLayers_.push_back("variance");
    elevationMapLayers_.push_back("horizontal_variance_x");
    elevationMapLayers_.push_back("horizontal_variance_y");
    elevationMapLayers_.push_back("horizontal_variance_xy");
    elevationMapLayers_.push_back("time");
  }
}

TraversabilityEstimation::~TraversabilityEstimation() {
rclcpp::shutdown(); 
}

bool TraversabilityEstimation::readParameters() {
  nodeHandle_->declare_parameter("use_raw_map", true);
  useRawMap_ = nodeHandle_->get_parameter("use_raw_map").as_bool();
  nodeHandle_->declare_parameter("submap_service", "/get_raw_submap");
  submapServiceName_ = nodeHandle_->get_parameter("submap_service").as_string();
  double updateRate;
  nodeHandle_->declare_parameter("min_update_rate", 5.0);
  updateRate = nodeHandle_->get_parameter("min_update_rate").as_double();
  if (updateRate != 0.0) {
    updateDuration_ = rclcpp::Duration::from_seconds(1.0 / updateRate);
  } else {
    updateDuration_ = rclcpp::Duration::from_seconds(0.0);
  }

  nodeHandle_->declare_parameter("image_topic", "/image_elevation");
  imageTopic_ = nodeHandle_->get_parameter("image_topic").as_string();
  nodeHandle_->declare_parameter("resolution", 0.03);
  imageResolution_ = nodeHandle_->get_parameter("resolution").as_double();
  nodeHandle_->declare_parameter("min_height", 0.0);
  imageMinHeight_ = nodeHandle_->get_parameter("min_height").as_double();
  nodeHandle_->declare_parameter("max_height", 1.0);
  imageMaxHeight_ = nodeHandle_->get_parameter("max_height").as_double();
  nodeHandle_->declare_parameter("image_position_x", 0.0);
  imagePosition_.x() = nodeHandle_->get_parameter("image_position_x").as_double();
  nodeHandle_->declare_parameter("image_position_y", 0.0);
  imagePosition_.y() = nodeHandle_->get_parameter("image_position_y").as_double();
  nodeHandle_->declare_parameter("robot_frame_id", "base_link");
  robotFrameId_ = nodeHandle_->get_parameter("robot_frame_id").as_string();
  nodeHandle_->declare_parameter("robot", "robot");
  robot_ = nodeHandle_->get_parameter("robot").as_string();
  nodeHandle_->declare_parameter("package", "traversability_estimation");
  package_ = nodeHandle_->get_parameter("package").as_string();
  nodeHandle_->declare_parameter("map_center_x", 0.0);
  nodeHandle_->declare_parameter("map_center_y", 0.0);
  grid_map::Position mapCenter;
  mapCenter.x() = nodeHandle_->get_parameter("map_center_x").as_double();
  mapCenter.y() = nodeHandle_->get_parameter("map_center_y").as_double();

  submapPoint_.header.frame_id = robotFrameId_;
  submapPoint_.point.x = mapCenter.x();
  submapPoint_.point.y = mapCenter.y();
  submapPoint_.point.z = 0.0;

  nodeHandle_->declare_parameter("map_length_x", 10.0);
  mapLength_.x() = nodeHandle_->get_parameter("map_length_x").as_double();
  nodeHandle_->declare_parameter("map_length_y", 10.0);
  mapLength_.y() = nodeHandle_->get_parameter("map_length_y").as_double();
  nodeHandle_->declare_parameter("footprint_yaw", M_PI_2);
  footprintYaw_ = nodeHandle_->get_parameter("footprint_yaw").as_double();
  nodeHandle_->declare_parameter("grid_map_to_initialize_traversability_map.enable", true);
  acceptGridMapToInitTraversabilityMap_ = nodeHandle_->get_parameter("grid_map_to_initialize_traversability_map.enable").as_bool();
  nodeHandle_->declare_parameter("grid_map_to_initialize_traversability_map.grid_map_topic_name", "elevation_map");
  gridMapToInitTraversabilityMapTopic_ = nodeHandle_->get_parameter("grid_map_to_initialize_traversability_map.grid_map_topic_name").as_string();

  RCLCPP_INFO(nodeHandle_->get_logger(), "TraversabilityEstimation: Parameters read successfully.");
  RCLCPP_INFO(nodeHandle_->get_logger(), "TraversabilityEstimation: useRawMap: %d", useRawMap_);
  RCLCPP_INFO(nodeHandle_->get_logger(), "TraversabilityEstimation: submapServiceName: %s", submapServiceName_.c_str());
  RCLCPP_INFO(nodeHandle_->get_logger(), "TraversabilityEstimation: updateDuration: %f", updateDuration_.seconds());
  RCLCPP_INFO(nodeHandle_->get_logger(), "TraversabilityEstimation: imageTopic: %s", imageTopic_.c_str());
  RCLCPP_INFO(nodeHandle_->get_logger(), "TraversabilityEstimation: imageResolution: %f", imageResolution_);
  RCLCPP_INFO(nodeHandle_->get_logger(), "TraversabilityEstimation: imageMinHeight: %f", imageMinHeight_);
  RCLCPP_INFO(nodeHandle_->get_logger(), "TraversabilityEstimation: imageMaxHeight: %f", imageMaxHeight_);
  RCLCPP_INFO(nodeHandle_->get_logger(), "TraversabilityEstimation: imagePosition: %f, %f", imagePosition_.x(), imagePosition_.y());
  RCLCPP_INFO(nodeHandle_->get_logger(), "TraversabilityEstimation: robotFrameId: %s", robotFrameId_.c_str());
  RCLCPP_INFO(nodeHandle_->get_logger(), "TraversabilityEstimation: robot: %s", robot_.c_str());
  RCLCPP_INFO(nodeHandle_->get_logger(), "TraversabilityEstimation: package: %s", package_.c_str());
  RCLCPP_INFO(nodeHandle_->get_logger(), "TraversabilityEstimation: mapCenter: %f, %f", mapCenter.x(), mapCenter.y());
  RCLCPP_INFO(nodeHandle_->get_logger(), "TraversabilityEstimation: mapLength: %f, %f", mapLength_.x(), mapLength_.y());
  RCLCPP_INFO(nodeHandle_->get_logger(), "TraversabilityEstimation: footprintYaw: %f", footprintYaw_);
  RCLCPP_INFO(nodeHandle_->get_logger(), "TraversabilityEstimation: acceptGridMapToInitTraversabilityMap: %d", acceptGridMapToInitTraversabilityMap_);
  RCLCPP_INFO(nodeHandle_->get_logger(), "TraversabilityEstimation: gridMapToInitTraversabilityMapTopic: %s", gridMapToInitTraversabilityMapTopic_.c_str());
  return true;
}

bool TraversabilityEstimation::loadElevationMap(const std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> request,
                                                std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response) {
  RCLCPP_INFO(nodeHandle_->get_logger(), "Loading Elevation Map");
  if (request->file_path.empty() || request->topic_name.empty()) {
    RCLCPP_WARN(nodeHandle_->get_logger(), "Fields 'file_path' and 'topic_name' in service request must be filled in.");
    response->success = static_cast<unsigned char>(false);
    return true;
  }
  grid_map::GridMap map;
  if (!grid_map::GridMapRosConverter::loadFromBag(request->file_path, request->topic_name, map)) {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "TraversabilityEstimation: Cannot find bag '%s' or topic '%s' of the elevation map!",
                 request->file_path.c_str(), request->topic_name.c_str());
    response->success = static_cast<unsigned char>(false);
  } else {
    map.setTimestamp(rclcpp::Clock().now().nanoseconds());
    if (!initializeTraversabilityMapFromGridMap(map)) {
      RCLCPP_ERROR(nodeHandle_->get_logger(), "TraversabilityEstimation: loadElevationMap: it was not possible to load elevation map from bag with path '%s' and topic '%s'.",
                   request->file_path.c_str(), request->topic_name.c_str());
      response->success = static_cast<unsigned char>(false);
    } else {
      response->success = static_cast<unsigned char>(true);
    }
  }

  return true;
}

void TraversabilityEstimation::imageCallback(const sensor_msgs::msg::Image::SharedPtr image) {
  if (!getImageCallback_) {
    grid_map::GridMapRosConverter::initializeFromImage(*image, imageResolution_, imageGridMap_, imagePosition_);
    RCLCPP_INFO(nodeHandle_->get_logger(), "Initialized map with size %f x %f m (%i x %i cells).", 
                imageGridMap_.getLength().x(), imageGridMap_.getLength().y(),
                imageGridMap_.getSize()(0), imageGridMap_.getSize()(1));
    imageGridMap_.add("upper_bound", 0.0);  // TODO: Add value for layers.
    imageGridMap_.add("lower_bound", 0.0);
    imageGridMap_.add("uncertainty_range", imageGridMap_.get("upper_bound") - imageGridMap_.get("lower_bound"));
    getImageCallback_ = true;
  }
  grid_map::GridMapRosConverter::addLayerFromImage(*image, "elevation", imageGridMap_, imageMinHeight_, imageMaxHeight_);
  auto elevationMapMessage = grid_map::GridMapRosConverter::toMessage(imageGridMap_);
  if (elevationMapMessage) {
    traversabilityMap_.setElevationMap(*elevationMapMessage);
  } else {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "Failed to convert GridMap to message.");
  }
}

void TraversabilityEstimation::updateTimerCallback() {
  updateTraversability();
}

bool TraversabilityEstimation::updateServiceCallback(const std::shared_ptr<grid_map_msgs::srv::GetGridMapInfo::Request> request,
                                                     std::shared_ptr<grid_map_msgs::srv::GetGridMapInfo::Response> response) {
  if (updateDuration_.seconds() == 0) {
    if (!updateTraversability()) {
      RCLCPP_ERROR(nodeHandle_->get_logger(), "Traversability Estimation: Cannot update traversability!");
      return false;
    }
  }
  // Wait until traversability map is computed.
  while (!traversabilityMap_.traversabilityMapInitialized()) {
    RCLCPP_INFO(nodeHandle_->get_logger(), "Traversability Estimation: Waiting for traversability map to be computed.");
    rclcpp::sleep_for(std::chrono::seconds(1));
  }
  grid_map::GridMap traversabilityMap = traversabilityMap_.getTraversabilityMap();
  response->header.frame_id = traversabilityMap_.getMapFrameId();
  response->header.stamp = nodeHandle_->now();
  response->info.resolution = traversabilityMap.getResolution();
  response->info.length_x = traversabilityMap.getLength()[0];
  response->info.length_y = traversabilityMap.getLength()[1];
  geometry_msgs::msg::Pose pose;
  grid_map::Position position = traversabilityMap.getPosition();
  pose.position.x = position[0];
  pose.position.y = position[1];
  pose.orientation.w = 1.0;
  response->info.pose = pose;
  return true;
}

bool TraversabilityEstimation::updateTraversability() {
  auto node_clock = nodeHandle_->get_clock();
  if (!getImageCallback_) {
    if (!submapClient_->wait_for_service(std::chrono::duration<double>(15.0))) {
      RCLCPP_WARN(nodeHandle_->get_logger(), "Service not available after waiting");
      return false;
    }
    RCLCPP_DEBUG(nodeHandle_->get_logger(), "Request sent to %s.", submapServiceName_.c_str());

    if (requestElevationMap(elevationMap)) {
      traversabilityMap_.setElevationMap(elevationMap);
      if (!traversabilityMap_.computeTraversability()) return false;
    } else {
      RCLCPP_WARN_THROTTLE(nodeHandle_->get_logger(), *node_clock, 2000, "Service not available after waiting");
      return false;
    }
  } else {
    if (!traversabilityMap_.computeTraversability()) return false;
  }

  return true;
}

bool TraversabilityEstimation::updateParameter(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  // Load parameters file.
  std::string path = ament_index_cpp::get_package_share_directory(package_);
  std::string path_filter_parameter = path + "/config/" + robot_ + "_filter_parameter.yaml";
  std::string path_footprint_parameter = path + "/config/" + robot_ + "_footprint_parameter.yaml";

  // Filter parameters
  std::string commandString = "ros2 param load " + path_filter_parameter + " /traversability_estimation";
  const char* command_filter = commandString.c_str();
  if (system(command_filter) != 0) {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "Can't update filter parameter.");
    return false;
  }

  // Footprint parameters
  commandString = "ros2 param load " + path_footprint_parameter + " /traversability_estimation";
  const char* command_footprint = commandString.c_str();
  if (system(command_footprint) != 0) {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "Can't update footprint parameter.");
    return false;
  }

  if (!traversabilityMap_.updateFilter()) return false;
  return true;
}

bool TraversabilityEstimation::requestElevationMap(grid_map_msgs::msg::GridMap& map) {
  submapPoint_.header.stamp = rclcpp::Time(0);
  geometry_msgs::msg::PointStamped submapPointTransformed;

  try {
    while (!tfBuffer_.canTransform(traversabilityMap_.getMapFrameId(), submapPoint_.header.frame_id, tf2::TimePointZero, tf2::durationFromSec(1.0))) {
      RCLCPP_INFO(nodeHandle_->get_logger(), "Waiting for transform to become available...");
      rclcpp::sleep_for(std::chrono::seconds(5));
    }
    tfBuffer_.transform(submapPoint_, submapPointTransformed, traversabilityMap_.getMapFrameId());
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "Failed to transform points: %s", ex.what());
    return false;
  }

  auto submapRequest = std::make_shared<grid_map_msgs::srv::GetGridMap::Request>();
  submapRequest->position_x = submapPointTransformed.point.x;
  submapRequest->position_y = submapPointTransformed.point.y;
  submapRequest->length_x = mapLength_.x();
  submapRequest->length_y = mapLength_.y();
  submapRequest->layers = elevationMapLayers_;

  while (!submapClient_->wait_for_service(std::chrono::seconds(5))) {
      if (!rclcpp::ok()) {
          RCLCPP_INFO(nodeHandle_->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return false;
      }
      RCLCPP_WARN(nodeHandle_->get_logger(), "Service not available after waiting");
  }

  auto future = submapClient_->async_send_request(submapRequest, std::bind(&TraversabilityEstimation::handle_response, this, std::placeholders::_1));
  
  if (future.valid()){
    return true;
  } else {
    RCLCPP_WARN(nodeHandle_->get_logger(), "Future is not valid");
    return false;
  }
}

void TraversabilityEstimation::handle_response(const rclcpp::Client<grid_map_msgs::srv::GetGridMap>::SharedFuture future) {
  try {
    auto submapResponse = future.get();
    elevationMap = submapResponse->map;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "Service call failed: %s", e.what());
  }
}

bool TraversabilityEstimation::traversabilityFootprint(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  if (!traversabilityMap_.traversabilityFootprint(footprintYaw_)) return false;

  return true;
}

bool TraversabilityEstimation::checkFootprintPath(const std::shared_ptr<traversability_msgs::srv::CheckFootprintPath::Request> request,
                                                  std::shared_ptr<traversability_msgs::srv::CheckFootprintPath::Response> response) {
  const int nPaths = request->path.size();
  auto node_clock_2 = nodeHandle_->get_clock();
  if (nPaths == 0) {
    RCLCPP_INFO_THROTTLE(nodeHandle_->get_logger(), *node_clock_2, 2000, "No footprint path available to check!");
    return false;
  }

  traversability_msgs::msg::TraversabilityResult result;
  traversability_msgs::msg::FootprintPath path;
  for (int j = 0; j < nPaths; j++) {
    path = request->path[j];
    if (!traversabilityMap_.checkFootprintPath(path, result, true)) return false;
    response->result.push_back(result);
  }

  return true;
}

bool TraversabilityEstimation::getTraversabilityMap(const std::shared_ptr<grid_map_msgs::srv::GetGridMap::Request> request,
                                          std::shared_ptr<grid_map_msgs::srv::GetGridMap::Response> response) {
  grid_map::Position requestedSubmapPosition(request->position_x, request->position_y);
  grid_map::Length requestedSubmapLength(request->length_x, request->length_y);
  grid_map_msgs::msg::GridMap msg;
  grid_map::GridMap map, subMap;
  map = traversabilityMap_.getTraversabilityMap();
  bool isSuccess;
  subMap = map.getSubmap(requestedSubmapPosition, requestedSubmapLength, isSuccess);
  if (isSuccess) {
    std::unique_ptr<grid_map_msgs::msg::GridMap> msg;

    if (request->layers.empty()) {
      msg = grid_map::GridMapRosConverter::toMessage(subMap);
    } else {
      std::vector<std::string> layers(request->layers.begin(), request->layers.end());
      msg = grid_map::GridMapRosConverter::toMessage(subMap, layers);
    }

    if (msg) {
      response->map = *msg;
    } else {
      RCLCPP_ERROR(nodeHandle_->get_logger(), "Failed to convert GridMap to message.");
      return false;
    }
  } else {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "Failed to get submap.");
    return false;
  }
  return true;
}

bool TraversabilityEstimation::saveToBag(const std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> request, std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response) {
  if (request->file_path.empty() || request->topic_name.empty()) {
    RCLCPP_WARN(nodeHandle_->get_logger(), "[TraversabilityEstimation::saveToBag]: Fields 'file_path' and 'topic_name' in service request must be filled in.");
    response->success = static_cast<unsigned char>(false);
    return true;
  }

  response->success = static_cast<unsigned char>(
      grid_map::GridMapRosConverter::saveToBag(traversabilityMap_.getTraversabilityMap(), request->file_path, request->topic_name));
  return true;
}

bool TraversabilityEstimation::initializeTraversabilityMapFromGridMap(const grid_map::GridMap& gridMap) {
  if (traversabilityMap_.traversabilityMapInitialized()) {
    // RCLCPP_WARN(nodeHandle_->get_logger(), "[TraversabilityEstimation::initializeTraversabilityMapFromGridMap]:Received grid map message cannot be used to initialize the traversability map, because current traversability map has been already initialized.");
    return false;
  }

  grid_map::GridMap mapWithCheckedLayers = gridMap;
  for (const auto& layer : elevationMapLayers_) {
    if (!mapWithCheckedLayers.exists(layer)) {
      mapWithCheckedLayers.add(layer, 0.0);
      RCLCPP_INFO(nodeHandle_->get_logger(), "[TraversabilityEstimation::initializeTraversabilityMapFromGridMap]: Added layer '%s'.", layer.c_str());
    }
  }
  RCLCPP_DEBUG(nodeHandle_->get_logger(), "Map frame id: %s", mapWithCheckedLayers.getFrameId().c_str());
  for (const auto& layer : mapWithCheckedLayers.getLayers()) {
    RCLCPP_DEBUG(nodeHandle_->get_logger(), "Map layer: %s", layer.c_str());
  }

  RCLCPP_DEBUG(nodeHandle_->get_logger(), "Map size: %f x %f m (%i x %i cells).",
               mapWithCheckedLayers.getLength().x(), mapWithCheckedLayers.getLength().y(),
               mapWithCheckedLayers.getSize()(0), mapWithCheckedLayers.getSize()(1));
  RCLCPP_DEBUG(nodeHandle_->get_logger(), "Map position: %f x %f m.",
               mapWithCheckedLayers.getPosition().x(), mapWithCheckedLayers.getPosition().y());
  RCLCPP_DEBUG(nodeHandle_->get_logger(), "Map resolution: %f m/cell.", mapWithCheckedLayers.getResolution());

  auto message = grid_map::GridMapRosConverter::toMessage(mapWithCheckedLayers);  
  if (message) {
    RCLCPP_DEBUG(nodeHandle_->get_logger(), "TraversabilityEstimation: initializeTraversabilityMapFromGridMap: Elevation map received.");
    traversabilityMap_.setElevationMap(*message);
    } 
  else {
      RCLCPP_WARN(nodeHandle_->get_logger(), "Failed to convert GridMap to message.");
      return false;
    }

  if (!traversabilityMap_.computeTraversability()) {
      RCLCPP_ERROR(nodeHandle_->get_logger(), "TraversabilityEstimation: initializeTraversabilityMapFromGridMap: cannot compute traversability.");
      return false;
  }
  return true;
}

void TraversabilityEstimation::gridMapToInitTraversabilityMapCallback(const grid_map_msgs::msg::GridMap& message) {
  grid_map::GridMap gridMap;
  grid_map::GridMapRosConverter::fromMessage(message, gridMap);
  if (initializeTraversabilityMapFromGridMap(gridMap)) {
    RCLCPP_INFO(nodeHandle_->get_logger(), "[TraversabilityEstimation::gridMapToInitTraversabilityMapCallback]:Traversability Map initialized using received grid map on topic '%s'.",
                gridMapToInitTraversabilityMapTopic_.c_str());
  }
  // if (!initializeTraversabilityMapFromGridMap(gridMap)) {
  //   RCLCPP_ERROR(nodeHandle_->get_logger(), "[TraversabilityEstimation::gridMapToInitTraversabilityMapCallback]:It was not possible to use received grid map message to initialize traversability map.");
  // } else {
  //   RCLCPP_INFO(nodeHandle_->get_logger(), "[TraversabilityEstimation::gridMapToInitTraversabilityMapCallback]:Traversability Map initialized using received grid map on topic '%s'.",
  //               gridMapToInitTraversabilityMapTopic_.c_str());
  // }
}

}  // namespace traversability_estimation
