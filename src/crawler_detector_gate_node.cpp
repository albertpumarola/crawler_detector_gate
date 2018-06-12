#include "crawler_detector_gate_node.h"

CrawlerDetectorGate::CrawlerDetectorGate(ros::NodeHandle& nh)
{
    _nh = nh;

    _pose_from_deep =
        _nh.subscribe("pose_from_deep_input", 1,
                      &CrawlerDetectorGate::grabPoseFromDeep, this);
    _pose_from_tags =
        _nh.subscribe("pose_from_tags_input", 1,
                      &CrawlerDetectorGate::grabPoseFromTags, this);

    _pose_pub = _nh.advertise<geometry_msgs::PoseStamped>("pose_output", 1000);
    _source_pub = _nh.advertise<std_msgs::String>("source_output", 1000);
    _marker_pub = _nh.advertise<visualization_msgs::Marker>("marker_output", 0);

    _nh.param<std::string>("cam_tf_name", _cam_tf_name, "cam");
    _nh.param<std::string>("gt_crawler_tf_name", _gt_crawler_tf_name,
                           "crawler");
    _nh.param<std::string>("estimated_crawler_tf_name",
                           _estimated_crawler_tf_name, "estim_crawler");
    _nh.param<std::string>("estimated_crawler_box_namespace",
                           _estimated_crawler_box_namespace,
                           "crawler/estim_box");
    _nh.param<std::string>("gt_crawler_box_namespace",
                           _gt_crawler_box_namespace, "crawler/gt_box");
    _nh.param<float>("max_publish_wait_time", _max_publish_wait_time, 0.1);
    _nh.param<float>("max_tags_delay_wait_time", _max_tags_delay_wait_factor,
                     2);

    _is_new_pose_from_deep = false;
    _is_new_pose_from_tags = false;

    _last_publish_stamp = ros::Time::now();
    _last_pose_from_tags_stamp = ros::Time::now();
    _delta_pose_from_tags_stamp = ros::Duration(0);
    _tags_delay_wait_factor = _max_tags_delay_wait_factor;
    _num_missed_tags_poses = 0;
};

void CrawlerDetectorGate::execute()
{
    ros::Duration time_since_pub = ros::Time::now() - _last_publish_stamp;

    if (_is_new_pose_from_tags) {
        publishEstimatedCrawlerPoseTF(_new_pose_from_tags);
        publishMarkers(_new_pose_from_tags, "box");

        publishSource("tags");
        publishCrawlerPose(_new_pose_from_tags);

        _num_missed_tags_poses = 0;
        _tags_delay_wait_factor = _max_tags_delay_wait_factor;

    } else if (_is_new_pose_from_deep) {
        ros::Duration wait_pose_from_tags_max_time = ros::Duration(
            _tags_delay_wait_factor * _delta_pose_from_tags_stamp.toSec());

        bool do_pub = time_since_pub > wait_pose_from_tags_max_time ||
                      time_since_pub > ros::Duration(_max_publish_wait_time);

        if (do_pub)
            if (_num_missed_tags_poses++ == 3) _tags_delay_wait_factor = 0.;

        if (do_pub) {
            publishSource("deep");
            publishCrawlerPose(_new_pose_from_deep);
        }
    }

    _is_new_pose_from_tags = false;
    _is_new_pose_from_deep = false;
}

void CrawlerDetectorGate::grabPoseFromDeep(
    const geometry_msgs::PoseStamped& msg)
{
    _new_pose_from_deep = msg;
    _is_new_pose_from_deep = true;
}

void CrawlerDetectorGate::grabPoseFromTags(
    const geometry_msgs::PoseStamped& msg)
{
    _new_pose_from_tags = msg;
    _is_new_pose_from_tags = true;

    ros::Time current_time = ros::Time::now();
    _delta_pose_from_tags_stamp = current_time - _last_pose_from_tags_stamp;
    _last_pose_from_tags_stamp = current_time;
}

void CrawlerDetectorGate::grabCrawlerGt()
{
    tf::StampedTransform transform;
    while (true) {
        ros::Time tnow = ros::Time(0);
        try {
            if (_lst.waitForTransform(_cam_tf_name, _gt_crawler_tf_name, tnow,
                                      ros::Duration(1.0))) {
                _lst.lookupTransform(_cam_tf_name, _gt_crawler_tf_name, tnow,
                                     transform);
                break;

            } else {
                ROS_WARN(
                    "[crawler_detector_gate]: Wait for world to crawler "
                    "transform failure.");
            }

        } catch (tf::TransformException ex) {
            ROS_ERROR(
                "[crawler_detector_gate]: World to crawler transform error: %s",
                ex.what());
        }
    }

    _crawler_gt_pose.header.stamp = ros::Time::now();
    _crawler_gt_pose.header.frame_id = _cam_tf_name;

    _crawler_gt_pose.pose.position.x = transform.getOrigin().x();
    _crawler_gt_pose.pose.position.y = transform.getOrigin().y();
    _crawler_gt_pose.pose.position.z = transform.getOrigin().z();

    _crawler_gt_pose.pose.orientation.x =
        transform.getRotation().getAxis().getX();
    _crawler_gt_pose.pose.orientation.y =
        transform.getRotation().getAxis().getY();
    _crawler_gt_pose.pose.orientation.z =
        transform.getRotation().getAxis().getZ();
    _crawler_gt_pose.pose.orientation.w = transform.getRotation().getW();
}

void CrawlerDetectorGate::publishEstimatedCrawlerPoseTF(
    const geometry_msgs::PoseStamped& new_pose_msg)
{
    tf::Vector3 position(new_pose_msg.pose.position.x,
                         new_pose_msg.pose.position.y,
                         new_pose_msg.pose.position.z);
    tf::Quaternion rotation(
        new_pose_msg.pose.orientation.x, new_pose_msg.pose.orientation.y,
        new_pose_msg.pose.orientation.z, new_pose_msg.pose.orientation.w);
    tf::Transform tf_crawler_pose(rotation, position);

    _br.sendTransform(tf::StampedTransform(tf_crawler_pose, ros::Time::now(),
                                           new_pose_msg.header.frame_id,
                                           _estimated_crawler_tf_name));
};

void CrawlerDetectorGate::publishCrawlerPose(
    const geometry_msgs::PoseStamped& new_pose_msg)
{
    _pose_pub.publish(new_pose_msg);
    _last_publish_stamp = ros::Time::now();
}

void CrawlerDetectorGate::publishSource(const std::string& source)
{
    std_msgs::String source_msg;
    source_msg.data = source;
    _source_pub.publish(source_msg);
}

void CrawlerDetectorGate::publishMarkers(
    const geometry_msgs::PoseStamped& new_pose_msg,
    const std::string& marker_type)
{
    if (marker_type == "box") 
        publishCrawlerBoxMarker(new_pose_msg, _estimated_crawler_box_namespace);

    grabCrawlerGt();
    publishCrawlerBoxMarker(_crawler_gt_pose, _gt_crawler_box_namespace);
}

void CrawlerDetectorGate::publishCrawlerBoxMarker(
    const geometry_msgs::PoseStamped& new_pose_msg, const std::string& ns)
{
    visualization_msgs::Marker box;
    box.header.stamp = new_pose_msg.header.stamp;
    box.header.frame_id = new_pose_msg.header.frame_id;
    box.ns = ns;
    box.id = 0;
    box.type = visualization_msgs::Marker::CUBE;
    box.pose.position.x = new_pose_msg.pose.position.x;
    box.pose.position.y = new_pose_msg.pose.position.y;
    box.pose.position.z = new_pose_msg.pose.position.z;
    box.pose.orientation.x = new_pose_msg.pose.orientation.x;
    box.pose.orientation.y = new_pose_msg.pose.orientation.y;
    box.pose.orientation.z = new_pose_msg.pose.orientation.z;
    box.pose.orientation.w = new_pose_msg.pose.orientation.w;
    box.scale.x = 0.2;
    box.scale.y = 0.2;
    box.scale.z = 0.1;
    box.action = visualization_msgs::Marker::ADD;
    box.color.a = 0.4f;
    box.lifetime = ros::Duration(1);	

    if (ns == _estimated_crawler_box_namespace)
        box.color.r = 1.f;
    else if (ns == _gt_crawler_box_namespace)
        box.color.b = 1.f;

    _marker_pub.publish(box);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh(ros::this_node::getName());
    ros::Rate loop_rate(30);

    CrawlerDetectorGate crawler_detector_gate(nh);

    while (ros::ok()) {
        crawler_detector_gate.execute();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
