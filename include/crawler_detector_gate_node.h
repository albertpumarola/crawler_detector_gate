#ifndef CRAWLER_DETECTOR_GATE_H
#define CRAWLER_DETECTOR_GATE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <assert.h>
#include <string>
#include <fstream>

class CrawlerDetectorGate
{
public:
    CrawlerDetectorGate(ros::NodeHandle& nh);
    void execute();

private:
    ros::NodeHandle _nh;
    ros::Subscriber _pose_from_deep;
    ros::Subscriber _pose_from_tags;
    ros::Publisher _pose_pub;
    ros::Publisher _source_pub;
    ros::Publisher _marker_pub;
    tf::TransformBroadcaster _br;
    tf::TransformListener _lst;

    std::string _cam_tf_name;
    std::string _gt_crawler_tf_name;
    std::string _estimated_crawler_tf_name;
    std::string _gt_crawler_box_namespace;
    std::string _estimated_crawler_box_namespace;

    bool _is_new_pose_from_deep;
    bool _is_new_pose_from_tags;
    geometry_msgs::PoseStamped _new_pose_from_deep;
    geometry_msgs::PoseStamped _new_pose_from_tags;
    geometry_msgs::PoseStamped _crawler_gt_pose;

    ros::Time _last_publish_stamp;
    ros::Time _last_pose_from_tags_stamp;
    ros::Duration _delta_pose_from_tags_stamp;
    float _max_publish_wait_time;
    float _max_tags_delay_wait_factor;
    float _tags_delay_wait_factor;
    int _num_missed_tags_poses;

    void grabPoseFromDeep(const geometry_msgs::PoseStamped& msg);
    void grabPoseFromTags(const geometry_msgs::PoseStamped& msg);
    void grabCrawlerGt();

    void publishCrawlerPose(const geometry_msgs::PoseStamped& new_pose_msg);
    void publishEstimatedCrawlerPoseTF(const geometry_msgs::PoseStamped& new_pose_msg);
    void publishSource(const std::string& source);

    void publishMarkers(const geometry_msgs::PoseStamped& new_pose_msg, const std::string& marker_type);
    void publishCrawlerBoxMarker(const geometry_msgs::PoseStamped& new_pose_msg, const std::string& ns);
};

#endif
