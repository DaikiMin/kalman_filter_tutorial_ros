#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>
#include <kalman_filter_tutorial_ros/kalman_filter_parameterConfig.h>
#include <iostream>
#include <kalman_filter_tutorial_ros/kalman_filter.hpp>

namespace kalman_filter_tutorial_ros {
    class Tracker {
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Publisher pub_marker_;
            ros::Subscriber sub_true_value_;
            ros::Subscriber sub_observed_value_;
            ros::Subscriber sub_observed_value_add_;
            ros::Timer timer_;

            std::unique_ptr<kalman_filter_tutorial_ros::KalmanFilter> kf_;
            visualization_msgs::Marker trajectory_;
            geometry_msgs::PointPtr true_value_msg_;
            geometry_msgs::PointPtr observed_value_msg_;

            bool exists_target_;

            dynamic_reconfigure::Server<kalman_filter_tutorial_ros::kalman_filter_parameterConfig>* server_;
            dynamic_reconfigure::Server<kalman_filter_tutorial_ros::kalman_filter_parameterConfig>::CallbackType f_;

            void callbackDynamicReconfigure(kalman_filter_tutorial_ros::kalman_filter_parameterConfig& config, uint32_t level);
            void callbackTrueValue( const geometry_msgs::PointConstPtr &msg );
            void callbackObservedValue( const geometry_msgs::PointConstPtr &msg );
            void callbackTimer( const ros::TimerEvent& e );

        public:
            Tracker( );
    };
}

void kalman_filter_tutorial_ros::Tracker::callbackDynamicReconfigure(kalman_filter_tutorial_ros::kalman_filter_parameterConfig& config, uint32_t level) {
    kf_->changeParameter( 1.0/config.fps, config.process_noise, config.system_noise );
}

void kalman_filter_tutorial_ros::Tracker::callbackTrueValue( const geometry_msgs::PointConstPtr &msg ) {
    *true_value_msg_ = *msg;
    return;
}

void kalman_filter_tutorial_ros::Tracker::callbackObservedValue( const geometry_msgs::PointConstPtr &msg ) {
    *observed_value_msg_ = *msg;
    return;
}

void kalman_filter_tutorial_ros::Tracker::callbackTimer( const ros::TimerEvent& e ) {
    std::cout << "\n=======================================================" << std::endl;
    ROS_INFO("true_value\t= %8.3f [m],\t%8.3f [m]", true_value_msg_->x, true_value_msg_->y );
    ROS_INFO("observed_value\t= %8.3f [m],\t%8.3f [m]", observed_value_msg_->x, observed_value_msg_->y );
    Eigen::Vector2f observed_value(  observed_value_msg_->x, observed_value_msg_->y );
    Eigen::Vector4f estimated_value( 0.0, 0.0, 0.0, 0.0 );
    if ( !exists_target_ ) {
        kf_->init( observed_value );
        exists_target_ = true;
    } else {
        kf_->compute( observed_value, &estimated_value );
    }
    ROS_INFO("estimated_value\t= %8.3f [m],\t%8.3f [m], \t%8.3f [m/s],\t%8.3f [m/s]", estimated_value[0], estimated_value[1], estimated_value[2], estimated_value[3] );

    geometry_msgs::Point position;
    position.x = estimated_value[0];
    position.y = estimated_value[1];
    position.z = 0.1;
    trajectory_.points.push_back( position );
    if ( trajectory_.points.size() > 200 ) trajectory_.points.erase(trajectory_.points.begin());
    trajectory_.header.stamp = ros::Time::now();
    pub_marker_.publish ( trajectory_ );
}

kalman_filter_tutorial_ros::Tracker::Tracker( ) : nh_(), pnh_("~") {
    pub_marker_ = nh_.advertise< visualization_msgs::Marker >( "/track_marker", 1 );
    sub_true_value_ = nh_.subscribe( "/true_value", 1, &kalman_filter_tutorial_ros::Tracker::callbackTrueValue, this );
    sub_observed_value_ = nh_.subscribe( "/observed_value", 1, &kalman_filter_tutorial_ros::Tracker::callbackObservedValue, this );
    timer_ = nh_.createTimer( ros::Duration(0.033), &kalman_filter_tutorial_ros::Tracker::callbackTimer, this );
    kf_.reset( new kalman_filter_tutorial_ros::KalmanFilter( 0.033, 1000, 1.0 ) );

    server_ = new dynamic_reconfigure::Server<kalman_filter_tutorial_ros::kalman_filter_parameterConfig>(pnh_);
    f_ = boost::bind(&kalman_filter_tutorial_ros::Tracker::callbackDynamicReconfigure, this, _1, _2);
    server_->setCallback(f_);

    true_value_msg_.reset( new geometry_msgs::Point );
    observed_value_msg_.reset( new geometry_msgs::Point );
    exists_target_ = false;
    trajectory_.header.frame_id = "map";
    trajectory_.header.stamp = ros::Time::now();
    trajectory_.ns = "trajectory";
    trajectory_.id =  1;
    trajectory_.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory_.action = visualization_msgs::Marker::ADD;
    trajectory_.scale.x = 0.05;
    trajectory_.color.r = 0.0; trajectory_.color.g = 1.0; trajectory_.color.b = 0.0; trajectory_.color.a = 1.0;
    trajectory_.pose.orientation.w = 1.0;
}

int main(int argc, char *argv[])  {
    ros::init(argc, argv, "tracker");
    kalman_filter_tutorial_ros::Tracker topic_sub;
    ros::spin();
    return 0;
}
