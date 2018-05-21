#include <pluginlib/class_list_macros.h>

#include <laser_odometry_core/laser_odometry_utils.h>
#include <laser_odometry_libpointmatcher/laser_odometry_libpointmatcher.h>

#include <tf_conversions/tf_eigen.h>

#include <fstream>

namespace laser_odometry {

bool LaserOdometryLibPointMatcher::configureImpl()
{
  std::string libpointmatcher_config_name;
  private_nh_.searchParam("libpointmatcher_config", libpointmatcher_config_name);

  if (private_nh_.hasParam(libpointmatcher_config_name))
  {
    std::string libpointmatcher_config;
    private_nh_.getParam(libpointmatcher_config_name, libpointmatcher_config);

    std::ifstream ifs(libpointmatcher_config.c_str());
    if (ifs.good())
    {
      icp_.loadFromYaml(ifs);
      ROS_INFO_STREAM("Load config from YAML file " << libpointmatcher_config);
    }
    else
    {
      ROS_ERROR_STREAM("Cannot load config from YAML file " << libpointmatcher_config);
      icp_.setDefault();
    }

    ifs.close();
  }
  else
  {
    ROS_WARN_STREAM("No config file specified, using default ICP chain.");
    icp_.setDefault();
  }

  kf_dist_linear_x_     = private_nh_.param("kf_dist_linear_x",  0.3);
  kf_dist_linear_y_     = private_nh_.param("kf_dist_linear_y",  0.3);
  kf_dist_angular_      = private_nh_.param("kf_dist_angular",   0.17);
  estimated_overlap_th_ = private_nh_.param("kf_overlap",        0.65);


  return true;
}

bool LaserOdometryLibPointMatcher::processImpl(const sensor_msgs::LaserScanConstPtr& scan_msg,
                                               const Transform& prediction)
{
  convert<sensor_msgs::LaserScan>(scan_msg, source_cloud_);

  return icp(source_cloud_, prediction);
}

bool LaserOdometryLibPointMatcher::processImpl(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                                               const Transform& prediction)
{
  convert<sensor_msgs::PointCloud2>(cloud_msg, source_cloud_);

  return icp(source_cloud_, prediction);
}

bool LaserOdometryLibPointMatcher::icp(const DataPointsPtr& src_cloud,
                                       const Transform& prediction)
{
  if (src_cloud->features.cols() == 0)
  {
    ROS_ERROR("No good points in the cloud");
    return false;
  }

  bool icp_valid = false;

  // Call ICP
  Matcher::TransformationParameters transform;
  try
  {
    transform = icp_(*src_cloud, *ref_cloud_, prediction.matrix());

    icp_valid = true;
  }
  catch (const Matcher::ConvergenceError& error)
  {
    ROS_ERROR_STREAM("ICP failed to converge: " << error.what());
    return false;
  }

  if (icp_valid)
  {
    increment_ = Eigen::Matrix<Scalar, 4, 4>(transform);

    /// @todo this damn cov is full of NaNs in 2D...
//    ROS_ERROR_STREAM("icp_.errorMinimizer->getCovariance()\n"
//                     << icp_.errorMinimizer->getCovariance());

//    increment_covariance_ = icp_.errorMinimizer->getCovariance();

    assert(increment_covariance_.rows()==6 &&
           increment_covariance_.cols()==6);
  }
  else
  {
    increment_.setIdentity();
    ROS_WARN("libpointmatcher could not align scans.");
  }

  return true;
}

void LaserOdometryLibPointMatcher::isKeyFrame()
{
  /// @todo what's best?
  std::swap(ref_cloud_, source_cloud_);
//  Matcher::swapDataPoints(*ref_cloud_, *source_cloud_);
}

bool LaserOdometryLibPointMatcher::initialize(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
  convert<sensor_msgs::LaserScan>(scan_msg, ref_cloud_);

  if (ref_cloud_->features.cols() == 0)
  {
    ROS_ERROR("initialize: No good points in the cloud");
    return false;
  }

  current_time_ = scan_msg->header.stamp;

  return true;
}

bool LaserOdometryLibPointMatcher::initialize(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  convert<sensor_msgs::PointCloud2>(cloud_msg, ref_cloud_);

  if (ref_cloud_->features.cols() == 0)
  {
    ROS_ERROR("initialize: No good points in the cloud");
    return false;
  }

  current_time_ = cloud_msg->header.stamp;

  return true;
}

bool LaserOdometryLibPointMatcher::isKeyFrame(const Transform& tf)
{

  if (std::abs(utils::getYaw(tf.rotation())) > kf_dist_angular_)
  {
    ROS_WARN_STREAM("Yaw too big. Max allowed: " << kf_dist_angular_ << " actual: " << std::abs(utils::getYaw(tf.rotation())));
    return false;
  }

  if (std::fabs(static_cast<float>(tf.translation()(0))) > kf_dist_linear_x_)
  {
    ROS_WARN_STREAM("X-dist too big. Max allowed: " << kf_dist_linear_x_ << " actual: " << std::fabs(static_cast<float>(tf.translation()(0))));
    return false;
  }

  if (std::fabs(static_cast<float>(tf.translation()(1))) > kf_dist_linear_y_)
  {
      ROS_WARN_STREAM("Y-dist too big. Max allowed: " << kf_dist_linear_y_ << " actual: " << std::fabs(static_cast<float>(tf.translation()(1))));
    return false;
  }

  if (icp_.errorMinimizer->getOverlap() < estimated_overlap_th_)
  {
    ROS_WARN_STREAM("Overlap key-frame: " << icp_.errorMinimizer->getOverlap());
    return false;
  }

  ROS_INFO_STREAM("This looks like a good keyframe!");

  return true;
}

OdomType LaserOdometryLibPointMatcher::odomType() const noexcept
{
  /// @todo @note depending on the icp_->errorMinimizer
  /// the returned covariance is either an actual covariance
  /// or identity.
  return OdomType::Odom3DCov;
}

} /* namespace laser_odometry */

PLUGINLIB_EXPORT_CLASS(laser_odometry::LaserOdometryLibPointMatcher, laser_odometry::LaserOdometryBase);
