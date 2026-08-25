#include "laser_line_extraction/line_extraction_ros.h"
#include <cmath>

namespace line_extraction
{

LineExtractionROS::LineExtractionROS() : Node("line_extraction_node"),
  data_cached_(false)
{
  loadParameters();

  line_publisher_ = this->create_publisher<laser_line_extraction::msg::LineSegmentList>(
    "line_segments", 1);

  scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, 1,
    [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
      this->laserScanCallback(msg);
    });

  if (pub_markers_)
  {
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "line_markers", 1);
  }

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(40),
    std::bind(&LineExtractionROS::run, this));
}

LineExtractionROS::~LineExtractionROS()
{
}

void LineExtractionROS::run()
{
  std::vector<Line> lines;
  line_extraction_.extractLines(lines);

  laser_line_extraction::msg::LineSegmentList msg;
  populateLineSegListMsg(lines, msg);
  line_publisher_->publish(msg);

  if (pub_markers_)
  {
    visualization_msgs::msg::Marker marker_msg;
    populateMarkerMsg(lines, marker_msg);
    marker_publisher_->publish(marker_msg);
  }
}

void LineExtractionROS::loadParameters()
{
  RCLCPP_DEBUG(this->get_logger(), "*************************************");
  RCLCPP_DEBUG(this->get_logger(), "PARAMETERS:");

  frame_id_ = this->declare_parameter<std::string>("frame_id", "laser");
  RCLCPP_DEBUG(this->get_logger(), "frame_id: %s", frame_id_.c_str());

  scan_topic_ = this->declare_parameter<std::string>("scan_topic", "scan");
  RCLCPP_DEBUG(this->get_logger(), "scan_topic: %s", scan_topic_.c_str());

  pub_markers_ = this->declare_parameter<bool>("publish_markers", false);
  RCLCPP_DEBUG(this->get_logger(), "publish_markers: %s", pub_markers_ ? "true" : "false");

  double bearing_std_dev = this->declare_parameter<double>("bearing_std_dev", 1e-3);
  line_extraction_.setBearingVariance(bearing_std_dev * bearing_std_dev);

  double range_std_dev = this->declare_parameter<double>("range_std_dev", 0.02);
  line_extraction_.setRangeVariance(range_std_dev * range_std_dev);

  double least_sq_angle_thresh = this->declare_parameter<double>("least_sq_angle_thresh", 1e-4);
  line_extraction_.setLeastSqAngleThresh(least_sq_angle_thresh);

  double least_sq_radius_thresh = this->declare_parameter<double>("least_sq_radius_thresh", 1e-4);
  line_extraction_.setLeastSqRadiusThresh(least_sq_radius_thresh);

  double max_line_gap = this->declare_parameter<double>("max_line_gap", 0.4);
  line_extraction_.setMaxLineGap(max_line_gap);

  double min_line_length = this->declare_parameter<double>("min_line_length", 0.5);
  line_extraction_.setMinLineLength(min_line_length);

  double min_range = this->declare_parameter<double>("min_range", 0.4);
  line_extraction_.setMinRange(min_range);

  double max_range = this->declare_parameter<double>("max_range", 10000.0);
  line_extraction_.setMaxRange(max_range);

  double min_split_dist = this->declare_parameter<double>("min_split_dist", 0.05);
  line_extraction_.setMinSplitDist(min_split_dist);

  double outlier_dist = this->declare_parameter<double>("outlier_dist", 0.05);
  line_extraction_.setOutlierDist(outlier_dist);

  int min_line_points = this->declare_parameter<int>("min_line_points", 9);
  line_extraction_.setMinLinePoints(static_cast<unsigned int>(min_line_points));

  RCLCPP_DEBUG(this->get_logger(), "*************************************");
}

void LineExtractionROS::populateLineSegListMsg(
  const std::vector<Line> &lines,
  laser_line_extraction::msg::LineSegmentList &line_list_msg)
{
  for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
  {
    laser_line_extraction::msg::LineSegment line_msg;
    line_msg.angle = cit->getAngle();
    line_msg.radius = cit->getRadius();
    auto cov = cit->getCovariance();
    line_msg.covariance = {(float)cov[0], (float)cov[1], (float)cov[2], (float)cov[3]};
    auto start = cit->getStart();
    line_msg.start = {(float)start[0], (float)start[1]};
    auto end = cit->getEnd();
    line_msg.end = {(float)end[0], (float)end[1]};
    line_list_msg.line_segments.push_back(line_msg);
  }
  line_list_msg.header.frame_id = frame_id_;
  line_list_msg.header.stamp = this->now();
}

void LineExtractionROS::populateMarkerMsg(
  const std::vector<Line> &lines,
  visualization_msgs::msg::Marker &marker_msg)
{
  marker_msg.ns = "line_extraction";
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_msg.scale.x = 0.1;
  marker_msg.color.r = 1.0;
  marker_msg.color.g = 0.0;
  marker_msg.color.b = 0.0;
  marker_msg.color.a = 1.0;
  for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
  {
    geometry_msgs::msg::Point p_start;
    p_start.x = cit->getStart()[0];
    p_start.y = cit->getStart()[1];
    p_start.z = 0;
    marker_msg.points.push_back(p_start);
    geometry_msgs::msg::Point p_end;
    p_end.x = cit->getEnd()[0];
    p_end.y = cit->getEnd()[1];
    p_end.z = 0;
    marker_msg.points.push_back(p_end);
  }
  marker_msg.header.frame_id = frame_id_;
  marker_msg.header.stamp = this->now();
}

void LineExtractionROS::cacheData(sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  std::vector<double> bearings, cos_bearings, sin_bearings;
  std::vector<unsigned int> indices;
  const std::size_t num_measurements = std::ceil(
      (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
  for (std::size_t i = 0; i < num_measurements; ++i)
  {
    const double b = scan_msg->angle_min + i * scan_msg->angle_increment;
    bearings.push_back(b);
    cos_bearings.push_back(cos(b));
    sin_bearings.push_back(sin(b));
    indices.push_back(i);
  }
  line_extraction_.setCachedData(bearings, cos_bearings, sin_bearings, indices);
  RCLCPP_DEBUG(this->get_logger(), "Data has been cached.");
}

void LineExtractionROS::laserScanCallback(sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
  if (!data_cached_)
  {
    cacheData(scan_msg);
    data_cached_ = true;
  }
  std::vector<double> scan_ranges_doubles(scan_msg->ranges.begin(), scan_msg->ranges.end());
  line_extraction_.setRangeData(scan_ranges_doubles);
}

} // namespace line_extraction