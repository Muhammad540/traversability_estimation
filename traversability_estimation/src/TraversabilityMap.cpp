/*
 * TraversabilityMap.cpp
 *
 *  Created on: Jun 09, 2014
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "traversability_estimation/TraversabilityMap.hpp"
#include "traversability_estimation/common.h"

// System
#include <algorithm>

// Grid Map
#include <grid_map_msgs/srv/get_grid_map.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/Polygon.hpp>

// ROS
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

// kindr
#include <kindr/Core>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// Param IO
#include <traversability_estimation/TraversabilityMap.hpp>

using namespace std;

namespace traversability_estimation {

TraversabilityMap::TraversabilityMap(rclcpp::Node::SharedPtr& nodeHandle)
    : nodeHandle_(nodeHandle),
      traversabilityType_("traversability"),
      slopeType_("traversability_slope"),
      stepType_("traversability_step"),
      roughnessType_("traversability_roughness"),
      robotSlopeType_("robot_slope"),
      filter_chain_("grid_map::GridMap"),
      zPosition_(0),
      elevationMapInitialized_(false),
      traversabilityMapInitialized_(false),
      checkForRoughness_(false),
      checkRobotInclination_(false) {
  RCLCPP_INFO(nodeHandle_->get_logger(), "Traversability Map started.");

  readParameters();
  traversabilityMapPublisher_ = nodeHandle_->create_publisher<grid_map_msgs::msg::GridMap>("traversability_map", 1);
  footprintPublisher_ = nodeHandle_->create_publisher<geometry_msgs::msg::PolygonStamped>("footprint_polygon", 1);
  untraversablePolygonPublisher_ = nodeHandle_->create_publisher<geometry_msgs::msg::PolygonStamped>("untraversable_polygon", 1);
  
}

TraversabilityMap::~TraversabilityMap() { rclcpp::shutdown(); }

bool TraversabilityMap::createLayers(bool useRawMap) {
  boost::recursive_mutex::scoped_lock scopedLockForElevationMap(elevationMapMutex_);
  RCLCPP_INFO(nodeHandle_->get_logger(), "Creating layers.");
  elevationMapLayers_.push_back("elevation");
  if (!useRawMap) {
    elevationMapLayers_.push_back("upper_bound");
    elevationMapLayers_.push_back("lower_bound");
  } else {
    elevationMapLayers_.push_back("variance");
    elevationMapLayers_.push_back("horizontal_variance_x");
    elevationMapLayers_.push_back("horizontal_variance_y");
    elevationMapLayers_.push_back("horizontal_variance_xy");
    elevationMapLayers_.push_back("time");
  }
  scopedLockForElevationMap.unlock();
  // TODO: Adapt map layers to traversability filters.
  boost::recursive_mutex::scoped_lock scopedLockForTraversabilityMap(traversabilityMapMutex_);
  traversabilityMapLayers_.push_back(traversabilityType_);
  traversabilityMapLayers_.push_back(slopeType_);
  traversabilityMapLayers_.push_back(stepType_);
  traversabilityMapLayers_.push_back(roughnessType_);
  scopedLockForTraversabilityMap.unlock();
  return true;
}

bool TraversabilityMap::readParameters() {
  nodeHandle_->declare_parameter<std::string>("map_frame_id", "map");
  nodeHandle_->declare_parameter<double>("footprint.traversability_default", 0.5);
  nodeHandle_->declare_parameter<bool>("footprint.verify_roughness_footprint", false);
  nodeHandle_->declare_parameter<bool>("footprint.verify_robot_inclination", false);
  nodeHandle_->declare_parameter<double>("max_gap_width", 0.3);
  nodeHandle_->declare_parameter<std::vector<double>>("footprint.footprint_polygon", std::vector<double>{});
  std::vector<double> footprint;
  nodeHandle_->get_parameter("footprint.footprint_polygon", footprint);
  if (footprint.empty()) {
    RCLCPP_WARN(nodeHandle_->get_logger(), "Traversability Map: No footprint polygon defined.");
  } else if (footprint.size() < 6) {
    RCLCPP_WARN(nodeHandle_->get_logger(), "Footprint polygon must consist of at least 3 points. Only %zu coordinates found.", footprint.size());
    footprintPoints_.clear();
  } else if (footprint.size() % 2 != 0) {
    RCLCPP_WARN(nodeHandle_->get_logger(), "Footprint polygon must have an even number of coordinates.");
    footprintPoints_.clear();
  } else {
    footprintPoints_.clear();
    geometry_msgs::msg::Point32 pt;
    pt.z = 0.0;
    for (size_t i = 0; i < footprint.size(); i += 2) {
      pt.x = footprint[i];
      pt.y = footprint[i + 1];
      footprintPoints_.push_back(pt);
    }
    for (const auto &point : footprintPoints_) {
        RCLCPP_INFO(nodeHandle_->get_logger(), "Footprint point: (%.2f, %.2f)", point.x, point.y);
      }
  }
  nodeHandle_->get_parameter("map_frame_id", mapFrameId_);
  nodeHandle_->get_parameter("footprint.traversability_default", traversabilityDefaultReadAtInit_);
  traversabilityDefaultReadAtInit_ = boundTraversabilityValue(traversabilityDefaultReadAtInit_);
  setDefaultTraversabilityUnknownRegions(traversabilityDefaultReadAtInit_);
  nodeHandle_->get_parameter("footprint.verify_roughness_footprint", checkForRoughness_);
  nodeHandle_->get_parameter("footprint.verify_robot_inclination", checkRobotInclination_);
  nodeHandle_->get_parameter("max_gap_width", maxGapWidth_);
  RCLCPP_INFO(nodeHandle_->get_logger(), "Traversability Map: Parameters read: ");
  RCLCPP_INFO(nodeHandle_->get_logger(), "map_frame_id: %s", mapFrameId_.c_str());
  RCLCPP_INFO(nodeHandle_->get_logger(), "footprint.traversability_default: %f", traversabilityDefaultReadAtInit_);
  RCLCPP_INFO(nodeHandle_->get_logger(), "footprint.verify_roughness_footprint: %d", checkForRoughness_);
  RCLCPP_INFO(nodeHandle_->get_logger(), "footprint.verify_robot_inclination: %d", checkRobotInclination_);
  RCLCPP_INFO(nodeHandle_->get_logger(), "max_gap_width: %f", maxGapWidth_);
  // Read filter parameters
  RCLCPP_INFO(nodeHandle_->get_logger(), "Reading filter parameters.");
  nodeHandle_->declare_parameter("traversability_filter_chain_parameter_name", std::string("traversability_map_filters"));
  nodeHandle_->get_parameter("traversability_filter_chain_parameter_name", filterChainParametersName_);
  nodeHandle_->declare_parameter<double>("traversability_map_filters.stepFilter.params.critical_value", 0.12);
  double criticalStepHeight_;
  nodeHandle_->get_parameter("traversability_map_filters.stepFilter.params.critical_value", criticalStepHeight_);
  RCLCPP_INFO(nodeHandle_->get_logger(), "Critical value for step filter: %f", criticalStepHeight_);
  // Configure filter chain
  if (filter_chain_.configure(
        filterChainParametersName_, 
        nodeHandle_->get_node_logging_interface(), 
        nodeHandle_->get_node_parameters_interface())) {
    RCLCPP_DEBUG(nodeHandle_->get_logger(), "Filter chain configured");
    return true;
  }
  else {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "Could not configure the filter chain!");
    return false;
  }
}

bool TraversabilityMap::setElevationMap(const grid_map_msgs::msg::GridMap& msg) {
  if (getMapFrameId() != msg.header.frame_id) {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "Received elevation map has frame_id = '%s', but an elevation map with frame_id = '%s' is expected.",
              msg.header.frame_id.c_str(), getMapFrameId().c_str());
    return false;
  }
  grid_map::GridMap elevationMap;
  grid_map::GridMapRosConverter::fromMessage(msg, elevationMap);
  boost::recursive_mutex::scoped_lock scopedLockForElevationMap(elevationMapMutex_);
  zPosition_ = msg.info.pose.position.z;
  for (auto& layer : elevationMapLayers_) {
    if (!elevationMap.exists(layer)) {
      RCLCPP_WARN(nodeHandle_->get_logger(), "Traversability Map: Can't set elevation map because there is no layer %s.", layer.c_str());
      return false;
    }
  }
  elevationMap_ = elevationMap;
  elevationMapInitialized_ = true;
  return true;
}

bool TraversabilityMap::setTraversabilityMap(const grid_map_msgs::msg::GridMap& msg) {
  grid_map::GridMap traversabilityMap;
  grid_map::GridMapRosConverter::fromMessage(msg, traversabilityMap);
  zPosition_ = msg.info.pose.position.z;
  boost::recursive_mutex::scoped_lock scopedLockForTraversabilityMap(traversabilityMapMutex_);
  for (auto& layer : traversabilityMapLayers_) {
    if (!traversabilityMap.exists(layer)) {
      RCLCPP_WARN(nodeHandle_->get_logger(), "Traversability Map: Can't set traversability map because there exists no layer %s.", layer.c_str());
      return false;
    }
  }
  traversabilityMap_ = traversabilityMap;
  traversabilityMapInitialized_ = true;
  return true;
}

void TraversabilityMap::publishTraversabilityMap() {
  if (!traversabilityMapPublisher_->get_subscription_count() < 1) {
    boost::recursive_mutex::scoped_lock scopedLockForTraversabilityMap(traversabilityMapMutex_);
    grid_map::GridMap traversabilityMapCopy = traversabilityMap_;
    scopedLockForTraversabilityMap.unlock();
    if (traversabilityMapCopy.exists("upper_bound") && traversabilityMapCopy.exists("lower_bound")) {
      traversabilityMapCopy.add("uncertainty_range", traversabilityMapCopy.get("upper_bound") - traversabilityMapCopy.get("lower_bound"));
    }

    std::unique_ptr<grid_map_msgs::msg::GridMap> mapMessage;
    mapMessage = grid_map::GridMapRosConverter::toMessage(traversabilityMapCopy);
    mapMessage->info.pose.position.z = zPosition_;
    RCLCPP_DEBUG(nodeHandle_->get_logger(), "Publishing traversability map.");
    traversabilityMapPublisher_->publish(std::move(mapMessage));
  }
}

grid_map::GridMap TraversabilityMap::getTraversabilityMap() {
  boost::recursive_mutex::scoped_lock scopedLockForTraversabilityMap(traversabilityMapMutex_);
  return traversabilityMap_;
}

bool TraversabilityMap::traversabilityMapInitialized() { 
  return traversabilityMapInitialized_; }

void TraversabilityMap::resetTraversabilityFootprintLayers() {
  boost::recursive_mutex::scoped_lock scopedLockForTraversabilityMap(traversabilityMapMutex_);
  if (traversabilityMap_.exists("step_footprint")) traversabilityMap_.clear("step_footprint");
  if (traversabilityMap_.exists("slope_footprint")) traversabilityMap_.clear("slope_footprint");
  if (traversabilityMap_.exists("traversability_footprint")) traversabilityMap_.clear("traversability_footprint");
}

bool TraversabilityMap::computeTraversability() {
  boost::recursive_mutex::scoped_lock scopedLockForTraversabilityMap(traversabilityMapMutex_);
  grid_map::GridMap traversabilityMapCopy = traversabilityMap_;
  scopedLockForTraversabilityMap.unlock();
  boost::recursive_mutex::scoped_lock scopedLockForElevationMap(elevationMapMutex_);
  grid_map::GridMap elevationMapCopy = elevationMap_;
  scopedLockForElevationMap.unlock();

  // Initialize timer.
  rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
  rclcpp::Time start = system_clock.now();

  if (elevationMapInitialized_) {
    if (!filter_chain_.update(elevationMapCopy, traversabilityMapCopy)) {
      RCLCPP_ERROR(nodeHandle_->get_logger(), "Traversability Estimation: Could not update the filter chain! No traversability computed!");
      traversabilityMapInitialized_ = false;
      return false;
    }
  } else {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "Traversability Estimation: Elevation map is not initialized!");
    traversabilityMapInitialized_ = false;
    return false;
  }
  traversabilityMapInitialized_ = true;
  traversabilityMapCopy.add("step_footprint");
  traversabilityMapCopy.add("slope_footprint");
  if (checkForRoughness_) traversabilityMapCopy.add("roughness_footprint");
  traversabilityMapCopy.add("traversability_footprint");

  scopedLockForTraversabilityMap.lock();
  traversabilityMap_ = traversabilityMapCopy;
  scopedLockForTraversabilityMap.unlock();
  publishTraversabilityMap();

  rclcpp::Time now = system_clock.now();
  RCLCPP_DEBUG(nodeHandle_->get_logger(), "Traversability map has been updated in %f s.", (now - start).seconds());
  return true;
}

bool TraversabilityMap::traversabilityFootprint(double footprintYaw) {
  if (!traversabilityMapInitialized_) return false;

  // Initialize timer.
  rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
  rclcpp::Time start = system_clock.now();


  boost::recursive_mutex::scoped_lock scopedLockForTraversabilityMap(traversabilityMapMutex_);
  traversabilityMap_.add("traversability_x");
  traversabilityMap_.add("traversability_rot");

  grid_map::Position position;
  grid_map::Polygon polygonX, polygonRot;

  RCLCPP_DEBUG(nodeHandle_->get_logger(), "footprint yaw: %f", footprintYaw);
  // Compute Orientation
  kindr::RotationQuaternionD xquat, rquat;
  kindr::AngleAxisD rotationAxis(footprintYaw, 0.0, 0.0, 1.0);
  rquat = rotationAxis * xquat;
  Eigen::Quaterniond orientationX = xquat.toImplementation();
  Eigen::Quaterniond orientationRot = rquat.toImplementation();

  for (grid_map::GridMapIterator iterator(traversabilityMap_); !iterator.isPastEnd(); ++iterator) {
    polygonX.removeVertices();
    polygonRot.removeVertices();
    traversabilityMap_.getPosition(*iterator, position);

    grid_map::Position3 positionToVertex, positionToVertexTransformedX, positionToVertexTransformedRot;
    Eigen::Translation<double, 3> toPosition;
    Eigen::Quaterniond orientation;

    toPosition.x() = position.x();
    toPosition.y() = position.y();
    toPosition.z() = 0.0;

    for (const auto& point : footprintPoints_) {
      positionToVertex.x() = point.x;
      positionToVertex.y() = point.y;
      positionToVertex.z() = point.z;
      positionToVertexTransformedX = toPosition * orientationX * positionToVertex;
      positionToVertexTransformedRot = toPosition * orientationRot * positionToVertex;

      grid_map::Position vertexX, vertexRot;
      vertexX.x() = positionToVertexTransformedX.x();
      vertexRot.x() = positionToVertexTransformedRot.x();
      vertexX.y() = positionToVertexTransformedX.y();
      vertexRot.y() = positionToVertexTransformedRot.y();
      polygonX.addVertex(vertexX);
      polygonRot.addVertex(vertexRot);
    }

    double traversability;
    if (isTraversable(polygonX, traversability))
      traversabilityMap_.at("traversability_x", *iterator) = traversability;
    else
      traversabilityMap_.at("traversability_x", *iterator) = 0.0;
    if (isTraversable(polygonRot, traversability))
      traversabilityMap_.at("traversability_rot", *iterator) = traversability;
    else
      traversabilityMap_.at("traversability_rot", *iterator) = 0.0;
  }
  scopedLockForTraversabilityMap.unlock();

  publishTraversabilityMap();

  rclcpp::Time now = system_clock.now();
  RCLCPP_INFO(nodeHandle_->get_logger(), "Traversability map has been updated in %f s.", (now - start).seconds());
  return true;
}

bool TraversabilityMap::traversabilityFootprint(const double& radius, const double& offset) {
  double traversability;
  grid_map::Position center;
  boost::recursive_mutex::scoped_lock scopedLockForTraversabilityMap(traversabilityMapMutex_);
  for (grid_map::GridMapIterator iterator(traversabilityMap_); !iterator.isPastEnd(); ++iterator) {
    traversabilityMap_.getPosition(*iterator, center);
    isTraversable(center, radius + offset, traversability, radius);
  }
  scopedLockForTraversabilityMap.unlock();
  publishTraversabilityMap();
  return true;
}

bool TraversabilityMap::checkFootprintPath(const traversability_msgs::msg::FootprintPath& path,
                                           traversability_msgs::msg::TraversabilityResult& result, const bool publishPolygons) {
  bool successfullyCheckedFootprint;
  if (!traversabilityMapInitialized_) {
    rclcpp::Clock::SharedPtr clock = nodeHandle_->get_clock();
    RCLCPP_WARN_THROTTLE(nodeHandle_->get_logger(), *clock, periodThrottledConsoleMessages, "Traversability Estimation: check Footprint path: Traversability map not yet initialized.");
    result.is_safe = static_cast<unsigned char>(false);
    return true;
  }

  const auto arraySize = path.poses.poses.size();
  if (arraySize == 0) {
    RCLCPP_WARN(nodeHandle_->get_logger(), "Traversability Estimation: This path has no poses to check!");
    result.is_safe = static_cast<unsigned char>(false);
    return false;
  }

  if (path.footprint.polygon.points.size() == 0) {
    successfullyCheckedFootprint = checkCircularFootprintPath(path, publishPolygons, result);
  } else {
    successfullyCheckedFootprint = checkPolygonalFootprintPath(path, publishPolygons, result);
  }

  return successfullyCheckedFootprint;
}

bool TraversabilityMap::checkCircularFootprintPath(const traversability_msgs::msg::FootprintPath& path, const bool publishPolygons,
                                                   traversability_msgs::msg::TraversabilityResult& result) {
  double radius = path.radius;
  double offset = 0.15;
  grid_map::Position start, end;
  const auto arraySize = path.poses.poses.size();
  const bool computeUntraversablePolygon = path.compute_untraversable_polygon;
  result.is_safe = static_cast<unsigned char>(false);
  result.traversability = 0.0;
  result.area = 0.0;
  double traversability = 0.0;
  double area = 0.0;
  grid_map::Polygon untraversablePolygon;
  auto robotHeight = computeMeanHeightFromPoses(path.poses.poses);

  for (int i = 0; i < arraySize; i++) {
    start = end;
    end.x() = path.poses.poses[i].position.x;
    end.y() = path.poses.poses[i].position.y;

    if (arraySize == 1) {
      if (checkRobotInclination_) {
        if (!checkInclination(end, end)) {
          return true;
        }
      }
      bool pathIsTraversable =
          isTraversable(end, radius + offset, computeUntraversablePolygon, traversability, untraversablePolygon, radius);
      if (publishPolygons) {
        grid_map::Polygon polygon = grid_map::Polygon::fromCircle(end, radius + offset);
        polygon.setFrameId(getMapFrameId());
        rclcpp::Clock::SharedPtr clock = nodeHandle_->get_clock();
        polygon.setTimestamp(clock->now().nanoseconds());
        publishFootprintPolygon(polygon);
        if (computeUntraversablePolygon) {
          publishUntraversablePolygon(untraversablePolygon, robotHeight);
        }
      }
      if (!pathIsTraversable) {
        // return such that default values in result - i.e. non traversable - are used.
        return true;
      }
      result.traversability = traversability;
    }

    if (arraySize > 1 && i > 0) {
      if (checkRobotInclination_) {
        if (!checkInclination(start, end)) {
          return true;
        }
      }
      double traversabilityTemp, traversabilitySum = 0.0;
      int nLine = 0;
      grid_map::Index startIndex, endIndex;
      boost::recursive_mutex::scoped_lock scopedLockForTraversabilityMap(traversabilityMapMutex_);
      traversabilityMap_.getIndex(start, startIndex);
      traversabilityMap_.getIndex(end, endIndex);
      int nSkip = 3;  // TODO: Remove magic number.
      grid_map::Polygon auxiliaryUntraversablePolygon;
      bool pathIsTraversable = true;
      for (grid_map::LineIterator lineIterator(traversabilityMap_, endIndex, startIndex); !lineIterator.isPastEnd(); ++lineIterator) {
        grid_map::Position center;
        traversabilityMap_.getPosition(*lineIterator, center);
        pathIsTraversable = pathIsTraversable && isTraversable(center, radius + offset, computeUntraversablePolygon, traversabilityTemp,
                                                               auxiliaryUntraversablePolygon, radius);

        if (publishPolygons && computeUntraversablePolygon && auxiliaryUntraversablePolygon.nVertices() > 0) {
          untraversablePolygon = grid_map::Polygon::convexHull(untraversablePolygon, auxiliaryUntraversablePolygon);
        }

        if (!pathIsTraversable && !computeUntraversablePolygon && !publishPolygons) {
          // return such that default values in result - i.e. non traversable - are used.
          return true;
        }

        traversabilitySum += traversabilityTemp;
        nLine++;
        for (int j = 0; j < nSkip; j++) {
          if (!lineIterator.isPastEnd()) {
            ++lineIterator;
          }
        }
      }
      scopedLockForTraversabilityMap.unlock();

      if (publishPolygons) {
        grid_map::Polygon polygon = grid_map::Polygon::fromCircle(end, radius + offset);
        polygon.setFrameId(getMapFrameId());
        rclcpp::Clock::SharedPtr clock = nodeHandle_->get_clock();
        polygon.setTimestamp(clock->now().nanoseconds());
        publishFootprintPolygon(polygon);
        if (computeUntraversablePolygon) {
          untraversablePolygon.setFrameId(auxiliaryUntraversablePolygon.getFrameId());
          untraversablePolygon.setTimestamp(auxiliaryUntraversablePolygon.getTimestamp());
          publishUntraversablePolygon(untraversablePolygon, robotHeight);
        }
      }

      if (pathIsTraversable) {
        traversability = traversabilitySum / (double)nLine;
        double lengthSegment, lengthPreviousPath, lengthPath;
        lengthSegment = (end - start).norm();
        if (i > 1) {
          lengthPreviousPath = lengthPath;
          lengthPath += lengthSegment;
          result.traversability = (lengthSegment * traversability + lengthPreviousPath * result.traversability) / lengthPath;
        } else {
          lengthPath = lengthSegment;
          result.traversability = traversability;
        }
      } else {
        // return such that default values in result - i.e. non traversable - are used.
        return true;
      }
    }
  }

  result.is_safe = static_cast<unsigned char>(true);
  return true;
}

bool TraversabilityMap::checkPolygonalFootprintPath(const traversability_msgs::msg::FootprintPath& path, const bool publishPolygons,
                                                    traversability_msgs::msg::TraversabilityResult& result) {
  grid_map::Position start, end;
  const auto arraySize = path.poses.poses.size();
  const bool computeUntraversablePolygon = path.compute_untraversable_polygon;
  result.is_safe = static_cast<unsigned char>(false);
  result.traversability = 0.0;
  result.area = 0.0;
  double traversability = 0.0;
  grid_map::Polygon untraversablePolygon;
  auto robotHeight = computeMeanHeightFromPoses(path.poses.poses);

  grid_map::Polygon polygon, polygon1, polygon2;
  polygon1.setFrameId(getMapFrameId());
  rclcpp::Clock::SharedPtr clock = nodeHandle_->get_clock();
  polygon1.setTimestamp(clock->now().nanoseconds());
  polygon2 = polygon1;
  for (int i = 0; i < arraySize; i++) {
    polygon1 = polygon2;
    start = end;
    polygon2.removeVertices();
    grid_map::Position3 positionToVertex, positionToVertexTransformed;
    Eigen::Translation<double, 3> toPosition;
    Eigen::Quaterniond orientation;

    toPosition.x() = path.poses.poses[i].position.x;
    toPosition.y() = path.poses.poses[i].position.y;
    toPosition.z() = path.poses.poses[i].position.z;
    orientation.x() = path.poses.poses[i].orientation.x;
    orientation.y() = path.poses.poses[i].orientation.y;
    orientation.z() = path.poses.poses[i].orientation.z;
    orientation.w() = path.poses.poses[i].orientation.w;
    end.x() = toPosition.x();
    end.y() = toPosition.y();

    for (const auto& point : path.footprint.polygon.points) {
      positionToVertex.x() = point.x;
      positionToVertex.y() = point.y;
      positionToVertex.z() = point.z;
      positionToVertexTransformed = toPosition * orientation * positionToVertex;

      grid_map::Position vertex;
      vertex.x() = positionToVertexTransformed.x();
      vertex.y() = positionToVertexTransformed.y();
      polygon2.addVertex(vertex);
    }

    if (path.conservative && i > 0) {
      grid_map::Vector startToEnd = end - start;
      vector<grid_map::Position> vertices1 = polygon1.getVertices();
      vector<grid_map::Position> vertices2 = polygon2.getVertices();
      for (const auto& vertex : vertices1) {
        polygon2.addVertex(vertex + startToEnd);
      }
      for (const auto& vertex : vertices2) {
        polygon1.addVertex(vertex - startToEnd);
      }
    }

    if (arraySize == 1) {
      polygon = polygon2;
      if (checkRobotInclination_) {
        if (!checkInclination(end, end)) return true;
      }
      bool pathIsTraversable = isTraversable(polygon, computeUntraversablePolygon, traversability, untraversablePolygon);

      if (publishPolygons) {
        publishFootprintPolygon(polygon);
        if (computeUntraversablePolygon) {
          publishUntraversablePolygon(untraversablePolygon, robotHeight);
        }
      }

      if (!pathIsTraversable) {
        // return such that default values in result - i.e. non traversable - are used.
        return true;
      }

      result.traversability = traversability;
      result.area = polygon.getArea();
    }

    if (arraySize > 1 && i > 0) {
      polygon = grid_map::Polygon::convexHull(polygon1, polygon2);
      polygon.setFrameId(getMapFrameId());
      rclcpp::Clock::SharedPtr clock = nodeHandle_->get_clock();
      polygon.setTimestamp(clock->now().nanoseconds());

      if (checkRobotInclination_) {
        if (!checkInclination(start, end)) {
          return true;
        }
      }
      bool pathIsTraversable = isTraversable(polygon, computeUntraversablePolygon, traversability, untraversablePolygon);

      if (publishPolygons) {
        publishFootprintPolygon(polygon, robotHeight);
        if (computeUntraversablePolygon) {
          publishUntraversablePolygon(untraversablePolygon, robotHeight);
        }
      }

      if (!pathIsTraversable) {
        // return such that default values in result - i.e. non traversable - are used.
        return true;
      }

      double areaPolygon, areaPrevious;
      if (i > 1) {
        areaPrevious = result.area;
        areaPolygon = polygon.getArea() - polygon1.getArea();
        result.area += areaPolygon;
        result.traversability = (areaPolygon * traversability + areaPrevious * result.traversability) / result.area;
      } else {
        result.area = polygon.getArea();
        result.traversability = traversability;
      }
    }
  }

  result.is_safe = static_cast<unsigned char>(true);
  return true;
}

bool TraversabilityMap::isTraversable(const grid_map::Polygon& polygon, double& traversability) {
  const bool computeUntraversablePolygon = false;
  grid_map::Polygon untraversablePolygon;
  return isTraversable(polygon, computeUntraversablePolygon, traversability, untraversablePolygon);
}

bool TraversabilityMap::isTraversable(const grid_map::Polygon& polygon, const bool& computeUntraversablePolygon, double& traversability,
                                      grid_map::Polygon& untraversablePolygon) {
  unsigned int nCells = 0;
  traversability = 0.0;
  bool pathIsTraversable = true;
  std::vector<grid_map::Position> untraversablePositions;
  // Iterate through polygon and check for traversability.
  boost::recursive_mutex::scoped_lock scopedLockForTraversabilityMap(traversabilityMapMutex_);
  for (grid_map::PolygonIterator polygonIterator(traversabilityMap_, polygon); !polygonIterator.isPastEnd(); ++polygonIterator) {
    bool currentPositionIsTraversale = isTraversableForFilters(*polygonIterator);

    if (!currentPositionIsTraversale) {
      pathIsTraversable = false;
      if (computeUntraversablePolygon) {
        grid_map::Position positionUntraversableCell;
        traversabilityMap_.getPosition(*polygonIterator, positionUntraversableCell);
        untraversablePositions.push_back(positionUntraversableCell);
      } else {
        return false;
      }
    } else {
      nCells++;
      if (!traversabilityMap_.isValid(*polygonIterator, traversabilityType_)) {
        traversability += traversabilityDefault_;
      } else {
        traversability += traversabilityMap_.at(traversabilityType_, *polygonIterator);
      }
    }
  }
  scopedLockForTraversabilityMap.unlock();

  if (pathIsTraversable) {
    // Handle cases of footprints outside of map.
    if (nCells == 0) {
      RCLCPP_DEBUG(nodeHandle_->get_logger(), "TraversabilityMap: isTraversable: No cells within polygon.");
      traversability = traversabilityDefault_;
      pathIsTraversable = traversabilityDefault_ != 0.0;
    } else {
      traversability /= nCells;
    }
  }

  if (computeUntraversablePolygon) {
    if (pathIsTraversable) {
      untraversablePolygon = grid_map::Polygon();  // empty untraversable polygon
    } else {
      untraversablePolygon = grid_map::Polygon::monotoneChainConvexHullOfPoints(untraversablePositions);
    }
    untraversablePolygon.setFrameId(getMapFrameId());
    rclcpp::Clock::SharedPtr clock = nodeHandle_->get_clock();
    untraversablePolygon.setTimestamp(clock->now().nanoseconds());
  }

  return pathIsTraversable;
}

bool TraversabilityMap::isTraversable(const grid_map::Position& center, const double& radiusMax, double& traversability,
                                      const double& radiusMin) {
  const bool computeUntraversablePolygon = false;
  grid_map::Polygon untraversablePolygon;
  return isTraversable(center, radiusMax, computeUntraversablePolygon, traversability, untraversablePolygon, radiusMin);
}

bool TraversabilityMap::isTraversable(const grid_map::Position& center, const double& radiusMax, const bool& computeUntraversablePolygon,
                                      double& traversability, grid_map::Polygon& untraversablePolygon, const double& radiusMin) {
  bool circleIsTraversable = true;
  std::vector<grid_map::Position> untraversablePositions;
  grid_map::Position positionUntraversableCell;
  untraversablePolygon = grid_map::Polygon();  // empty untraversable polygon
  // Handle cases of footprints outside of map.
  boost::recursive_mutex::scoped_lock scopedLockForTraversabilityMap(traversabilityMapMutex_);
  if (!traversabilityMap_.isInside(center)) {
    traversability = traversabilityDefault_;
    circleIsTraversable = traversabilityDefault_ != 0.0;
    if (computeUntraversablePolygon && !circleIsTraversable) {
      untraversablePolygon = grid_map::Polygon::fromCircle(center, radiusMax);
    }
  } else {
    // Footprints inside map.
    // Get index of center position.
    grid_map::Index indexCenter;
    traversabilityMap_.getIndex(center, indexCenter);
    if (traversabilityMap_.isValid(indexCenter, "traversability_footprint")) {
      traversability = traversabilityMap_.at("traversability_footprint", indexCenter);
      circleIsTraversable = traversability != 0.0;
      if (computeUntraversablePolygon && !circleIsTraversable) {
        untraversablePolygon = grid_map::Polygon::fromCircle(center, radiusMax);
      }
    } else {
      // Non valid (non finite traversability)
      int nCells = 0;
      traversability = 0.0;

      // Iterate through polygon and check for traversability.
      double maxUntraversableRadius = 0.0;
      bool traversableRadiusBiggerMinRadius = false;
      for (grid_map::SpiralIterator iterator(traversabilityMap_, center, radiusMax);
           !iterator.isPastEnd() && !traversableRadiusBiggerMinRadius; ++iterator) {
        const bool currentPositionIsTraversale = isTraversableForFilters(*iterator);
        if (!currentPositionIsTraversale) {
          const auto untraversableRadius = iterator.getCurrentRadius();
          maxUntraversableRadius = std::max(maxUntraversableRadius, untraversableRadius);

          if (radiusMin == 0.0) {
            traversabilityMap_.at("traversability_footprint", indexCenter) = 0.0;
            circleIsTraversable = false;
            traversabilityMap_.getPosition(*iterator, positionUntraversableCell);
            untraversablePositions.push_back(positionUntraversableCell);
          } else {
            if (untraversableRadius <= radiusMin) {
              traversabilityMap_.at("traversability_footprint", indexCenter) = 0.0;
              circleIsTraversable = false;
              traversabilityMap_.getPosition(*iterator, positionUntraversableCell);
              untraversablePositions.push_back(positionUntraversableCell);
            } else if (circleIsTraversable) {  // if circleIsTraversable is not changed by any previous loop
              auto factor = ((untraversableRadius - radiusMin) / (radiusMax - radiusMin) + 1.0) / 2.0;
              traversability *= factor / nCells;
              traversabilityMap_.at("traversability_footprint", indexCenter) = static_cast<float>(traversability);
              circleIsTraversable = true;
              traversableRadiusBiggerMinRadius = true;
            }
          }

          if (!computeUntraversablePolygon) {
            // Do not keep on checking, one cell is already non-traversable.
            return false;
          }
        } else {
          nCells++;
          if (!traversabilityMap_.isValid(*iterator, traversabilityType_)) {
            traversability += traversabilityDefault_;
          } else {
            traversability += traversabilityMap_.at(traversabilityType_, *iterator);
          }
        }
      }

      if (computeUntraversablePolygon && !circleIsTraversable) {
        untraversablePolygon = grid_map::Polygon::monotoneChainConvexHullOfPoints(untraversablePositions);
      }

      if (circleIsTraversable) {
        traversability /= nCells;
        traversabilityMap_.at("traversability_footprint", indexCenter) = static_cast<float>(traversability);
      }
    }
  }
  scopedLockForTraversabilityMap.unlock();

  if (computeUntraversablePolygon) {
    untraversablePolygon.setFrameId(getMapFrameId());
    rclcpp::Clock::SharedPtr clock = nodeHandle_->get_clock();
    untraversablePolygon.setTimestamp(clock->now().nanoseconds());
  }

  return circleIsTraversable;
}

bool TraversabilityMap::checkInclination(const grid_map::Position& start, const grid_map::Position& end) {
  boost::recursive_mutex::scoped_lock scopedLockForTraversabilityMap(traversabilityMapMutex_);
  if (end == start) {
    if (traversabilityMap_.atPosition(robotSlopeType_, start) == 0.0) return false;
  } else {
    grid_map::Index startIndex, endIndex;
    traversabilityMap_.getIndex(start, startIndex);
    traversabilityMap_.getIndex(end, endIndex);
    for (grid_map::LineIterator lineIterator(traversabilityMap_, startIndex, endIndex); !lineIterator.isPastEnd(); ++lineIterator) {
      if (!traversabilityMap_.isValid(*lineIterator, robotSlopeType_)) continue;
      if (traversabilityMap_.at(robotSlopeType_, *lineIterator) == 0.0) return false;
    }
  }
  return true;
}

bool TraversabilityMap::updateFilter() {
  // Reconfigure filter chain.
  filter_chain_.clear();
  if (!filter_chain_.configure("traversability_map_filters",
      nodeHandle_->get_node_logging_interface(),
      nodeHandle_->get_node_parameters_interface())) {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "Could not configure the filter chain!");
    return false;
  }
  return true;
}

bool TraversabilityMap::isTraversableForFilters(const grid_map::Index& indexStep) {
  boost::recursive_mutex::scoped_lock scopedLockForTraversabilityMap(traversabilityMapMutex_);
  bool currentPositionIsTraversale = true;
  if (checkForSlope(indexStep)) {
    if (checkForStep(indexStep)) {
      if (checkForRoughness_) {
        if (!checkForRoughness(indexStep)) {
          currentPositionIsTraversale = false;
        }
      }
    } else {
      currentPositionIsTraversale = false;
    }
  } else {
    currentPositionIsTraversale = false;
  }

  return currentPositionIsTraversale;
}

bool TraversabilityMap::checkForStep(const grid_map::Index& indexStep) {
  boost::recursive_mutex::scoped_lock scopedLockForTraversabilityMap(traversabilityMapMutex_);
  if (traversabilityMap_.at(stepType_, indexStep) == 0.0) {
    if (!traversabilityMap_.isValid(indexStep, "step_footprint")) {
      double windowRadiusStep = 2.5 * traversabilityMap_.getResolution();  // 0.075;

      vector<grid_map::Index> indices;
      grid_map::Position center;
      traversabilityMap_.getPosition(indexStep, center);
      double height = traversabilityMap_.at("elevation", indexStep);
      for (grid_map::CircleIterator circleIterator(traversabilityMap_, center, windowRadiusStep); !circleIterator.isPastEnd();
           ++circleIterator) {
        if (traversabilityMap_.at("elevation", *circleIterator) > criticalStepHeight_ + height &&
            traversabilityMap_.at(stepType_, *circleIterator) == 0.0)
          indices.push_back(*circleIterator);
      }
      if (indices.empty()) indices.push_back(indexStep);
      for (auto& index : indices) {
        grid_map::Length subMapLength(2.5 * traversabilityMap_.getResolution(), 2.5 * traversabilityMap_.getResolution());
        grid_map::Position subMapPos;
        bool isSuccess;
        traversabilityMap_.getPosition(index, subMapPos);
        grid_map::Vector toCenter = center - subMapPos;
        grid_map::GridMap subMap = traversabilityMap_.getSubmap(subMapPos, subMapLength, isSuccess);
        if (!isSuccess) {
          RCLCPP_WARN(nodeHandle_->get_logger(), "Traversability map: Check for step window could not retrieve submap.");
          traversabilityMap_.at("step_footprint", indexStep) = 0.0;
          return false;
        }
        height = traversabilityMap_.at("elevation", index);
        for (grid_map::GridMapIterator subMapIterator(subMap); !subMapIterator.isPastEnd(); ++subMapIterator) {
          if (subMap.at(stepType_, *subMapIterator) == 0.0 && subMap.at("elevation", *subMapIterator) < height - criticalStepHeight_) {
            grid_map::Position pos;
            subMap.getPosition(*subMapIterator, pos);
            grid_map::Vector vec = pos - subMapPos;
            if (vec.norm() < 0.025) continue;
            if (toCenter.norm() > 0.025) {
              if (toCenter.dot(vec) < 0.0) continue;
            }
            pos = subMapPos + vec;
            while ((pos - subMapPos + vec).norm() < maxGapWidth_ && traversabilityMap_.isInside(pos + vec)) pos += vec;
            grid_map::Index endIndex;
            traversabilityMap_.getIndex(pos, endIndex);
            bool gapStart = false;
            bool gapEnd = false;
            for (grid_map::LineIterator lineIterator(traversabilityMap_, index, endIndex); !lineIterator.isPastEnd(); ++lineIterator) {
              if (traversabilityMap_.at("elevation", *lineIterator) > height + criticalStepHeight_) {
                traversabilityMap_.at("step_footprint", indexStep) = 0.0;
                return false;
              }
              if (traversabilityMap_.at("elevation", *lineIterator) < height - criticalStepHeight_ ||
                  !traversabilityMap_.isValid(*lineIterator, "elevation")) {
                gapStart = true;
              } else if (gapStart) {
                gapEnd = true;
                break;
              }
            }
            if (gapStart && !gapEnd) {
              traversabilityMap_.at("step_footprint", indexStep) = 0.0;
              return false;
            }
          }
        }
      }
      traversabilityMap_.at("step_footprint", indexStep) = 1.0;
    } else if (traversabilityMap_.at("step_footprint", indexStep) == 0.0) {
      return false;
    }
  }
  return true;
}

bool TraversabilityMap::checkForSlope(const grid_map::Index& index) {
  boost::recursive_mutex::scoped_lock scopedLockForTraversabilityMap(traversabilityMapMutex_);
  if (traversabilityMap_.at(slopeType_, index) == 0.0) {
    if (!traversabilityMap_.isValid(index, "slope_footprint")) {
      double windowRadius = 3.0 * traversabilityMap_.getResolution();  // TODO: read this as a parameter?
      double criticalLength = maxGapWidth_ / 3.0;
      int nSlopesCritical = floor(2 * windowRadius * criticalLength / pow(traversabilityMap_.getResolution(), 2));

      // Requested position (center) of circle in map.
      grid_map::Position center;
      traversabilityMap_.getPosition(index, center);
      int nSlopes = 0;
      for (grid_map::CircleIterator circleIterator(traversabilityMap_, center, windowRadius); !circleIterator.isPastEnd();
           ++circleIterator) {
        if (traversabilityMap_.at(slopeType_, *circleIterator) == 0.0) nSlopes++;
        if (nSlopes > nSlopesCritical) {
          traversabilityMap_.at("slope_footprint", index) = 0.0;
          return false;
        }
      }
      traversabilityMap_.at("slope_footprint", index) = 1.0;
    } else if (traversabilityMap_.at("slope_footprint", index) == 0.0) {
      return false;
    }
  }
  return true;
}

bool TraversabilityMap::checkForRoughness(const grid_map::Index& index) {
  boost::recursive_mutex::scoped_lock scopedLockForTraversabilityMap(traversabilityMapMutex_);
  if (traversabilityMap_.at(roughnessType_, index) == 0.0) {
    if (!traversabilityMap_.isValid(index, "roughness_footprint")) {
      double windowRadius = 3.0 * traversabilityMap_.getResolution();  // TODO: read this as a parameter?
      double criticalLength = maxGapWidth_ / 3.0;
      int nRoughnessCritical = floor(1.5 * windowRadius * criticalLength / pow(traversabilityMap_.getResolution(), 2));

      // Requested position (center) of circle in map.
      grid_map::Position center;
      traversabilityMap_.getPosition(index, center);
      int nRoughness = 0;
      for (grid_map::CircleIterator circleIterator(traversabilityMap_, center, windowRadius); !circleIterator.isPastEnd();
           ++circleIterator) {
        if (traversabilityMap_.at(roughnessType_, *circleIterator) == 0.0) nRoughness++;
        if (nRoughness > nRoughnessCritical) {
          traversabilityMap_.at("roughness_footprint", index) = 0.0;
          return false;
        }
      }
      traversabilityMap_.at("roughness_footprint", index) = 1.0;
    } else if (traversabilityMap_.at("roughness_footprint", index) == 0.0) {
      return false;
    }
  }
  return true;
}

void TraversabilityMap::publishFootprintPolygon(const grid_map::Polygon& polygon, double zPosition) {
  if (footprintPublisher_->get_subscription_count() < 1) return;
  geometry_msgs::msg::PolygonStamped polygonMsg;
  grid_map::PolygonRosConverter::toMessage(polygon, polygonMsg);
  for (int i = 0; i < polygonMsg.polygon.points.size(); i++) {
    polygonMsg.polygon.points.at(i).z = zPosition;
  }
  footprintPublisher_->publish(polygonMsg);
}

void TraversabilityMap::publishUntraversablePolygon(const grid_map::Polygon& untraversablePolygon, double zPosition) {
  if (untraversablePolygonPublisher_->get_subscription_count() < 1 || untraversablePolygon.nVertices() == 0) {
    return;
  }
  geometry_msgs::msg::PolygonStamped polygonMsg;
  grid_map::PolygonRosConverter::toMessage(untraversablePolygon, polygonMsg);
  for (auto& polygon_point : polygonMsg.polygon.points) {
    polygon_point.z = static_cast<float>(zPosition);
  }
  untraversablePolygonPublisher_->publish(polygonMsg);
}

std::string TraversabilityMap::getMapFrameId() const { 
  return mapFrameId_; }

double TraversabilityMap::getDefaultTraversabilityUnknownRegions() const { 
  return traversabilityDefault_; }

void TraversabilityMap::setDefaultTraversabilityUnknownRegions(const double& defaultTraversability) {
  traversabilityDefault_ = boundTraversabilityValue(defaultTraversability);
}

void TraversabilityMap::restoreDefaultTraversabilityUnknownRegionsReadAtInit() {
  setDefaultTraversabilityUnknownRegions(traversabilityDefaultReadAtInit_);
}

double TraversabilityMap::boundTraversabilityValue(const double& traversabilityValue) const {
  if (traversabilityValue > traversabilityMaxValue) {
    RCLCPP_WARN(nodeHandle_->get_logger(), "Passed traversability value (%f) is higher than max allowed value (%f). It is set equal to the max.", traversabilityValue, traversabilityMaxValue);
    return traversabilityMaxValue;
  }
  if (traversabilityValue < traversabilityMinValue) {
    RCLCPP_WARN(nodeHandle_->get_logger(), "Passed traversability value (%f) is lower than min allowed value (%f). It is set equal to the min.", traversabilityValue, traversabilityMinValue);
    return traversabilityMinValue;
  }
  return traversabilityValue;
}

bool TraversabilityMap::mapHasValidTraversabilityAt(double x, double y) const {
  grid_map::Position positionToCheck(x, y);
  grid_map::Index indexToCheck;
  boost::recursive_mutex::scoped_lock scopedLockForTraversabilityMap(traversabilityMapMutex_);
  auto indexObtained = traversabilityMap_.getIndex(positionToCheck, indexToCheck);
  if (!indexObtained) {
    RCLCPP_ERROR(nodeHandle_->get_logger(), "It was not possible to get index of the position (%f, %f) in the current grid_map representation of the traversability map.", x, y);
    return false;
  }

  return traversabilityMap_.isValid(indexToCheck, traversabilityType_);
}

}  // namespace traversability_estimation
