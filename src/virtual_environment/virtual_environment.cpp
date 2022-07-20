#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include<bits/stdc++.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <dynamic_reconfigure/server.h>
#include <kalman_filter_tutorial_ros/observation_parameterConfig.h>

namespace kalman_filter_tutorial_ros{
    class VirtualEnvironment {
        private :
            ros::NodeHandle nh_;
            ros::NodeHandle pnh_;
            ros::Publisher pub_marker_;
            ros::Publisher pub_observed_value_;
            ros::Publisher pub_true_value_;
            ros::Subscriber sub_teleop_;
            geometry_msgs::Point tgt_pt_;
            double tgt_theta_;
            std::unique_ptr<ros::Rate> rate_;
            std::unique_ptr<std::normal_distribution<double>> dist_;

            dynamic_reconfigure::Server<kalman_filter_tutorial_ros::observation_parameterConfig>* server_;
            dynamic_reconfigure::Server<kalman_filter_tutorial_ros::observation_parameterConfig>::CallbackType f_;

            void callbackDynamicReconfigure(kalman_filter_tutorial_ros::observation_parameterConfig& config, uint32_t level);
            void callbackTarget ( const geometry_msgs::TwistConstPtr &msg );
            void displaySensorMarker ( );

        public :
            VirtualEnvironment ( );
            void pubData (  );
    };
}

void kalman_filter_tutorial_ros::VirtualEnvironment::callbackDynamicReconfigure(kalman_filter_tutorial_ros::observation_parameterConfig& config, uint32_t level) {
    rate_.reset( new ros::Rate(config.fps) );
    dist_.reset( new std::normal_distribution<double>( 0.0, config.observation_noise ) );
}

void kalman_filter_tutorial_ros::VirtualEnvironment::callbackTarget ( const geometry_msgs::TwistConstPtr &msg ) {
    tgt_theta_ += 0.05 * msg->angular.z ;
    if ( tgt_theta_ > M_PI )    tgt_theta_ =tgt_theta_ - 2 * M_PI;
    if ( tgt_theta_ < - M_PI )  tgt_theta_ =tgt_theta_ + 2 * M_PI;

    if ( msg->linear.y == 0.0 ) {
        tgt_pt_.x += 0.05 * msg->linear.x * std::cos( tgt_theta_ );
        tgt_pt_.y += 0.05 * msg->linear.x * std::sin( tgt_theta_ );
    } else {
        double ang = std::atan2( msg->linear.y, msg->linear.x );
        double dist = std::hypotf( msg->linear.x, msg->linear.y );
        tgt_pt_.x += 0.05 * dist * std::cos( tgt_theta_ + ang );
        tgt_pt_.y += 0.05 * dist * std::sin( tgt_theta_ + ang );
    }

    return;
}

kalman_filter_tutorial_ros::VirtualEnvironment::VirtualEnvironment ( ) : nh_(), pnh_("~") {
    tgt_pt_.x = 0.0;
    tgt_pt_.y = 0.0;
    tgt_theta_ = 0.0;
    pub_marker_ = nh_.advertise< visualization_msgs::MarkerArray >( "/target_marker", 1 );
    pub_observed_value_ = nh_.advertise< geometry_msgs::Point >( "/observed_value", 1 );
    pub_true_value_ = nh_.advertise< geometry_msgs::Point >( "/true_value", 1 );
    sub_teleop_ = nh_.subscribe( "/cmd_vel_mux/input/teleop", 10, &VirtualEnvironment::callbackTarget, this );

    server_ = new dynamic_reconfigure::Server<kalman_filter_tutorial_ros::observation_parameterConfig>(pnh_);
    f_ = boost::bind(&kalman_filter_tutorial_ros::VirtualEnvironment::callbackDynamicReconfigure, this, _1, _2);
    server_->setCallback(f_);
}

void kalman_filter_tutorial_ros::VirtualEnvironment::pubData (  ) {
    static tf::TransformBroadcaster br;
    visualization_msgs::Marker trajectory, observed;
    trajectory.header.frame_id = observed.header.frame_id = "map";
    trajectory.header.stamp = observed.header.stamp = ros::Time::now();
    trajectory.ns = "trajectory"; observed.ns = "observed";
    trajectory.id = observed.id = 1;
    trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    observed.type = visualization_msgs::Marker::SPHERE;
    trajectory.action = observed.action = visualization_msgs::Marker::ADD;
    trajectory.scale.x = 0.2;
    observed.scale.x = 0.2; observed.scale.y = 0.2; observed.scale.z = 0.2;
    trajectory.color.r = 1.0; trajectory.color.g = 0.0; trajectory.color.b = 0.0; trajectory.color.a = 1.0;
    observed.color.r = 0.0; observed.color.g = 0.0; observed.color.b = 1.0; observed.color.a = 1.0;
    trajectory.pose.orientation.w = observed.pose.orientation.w = 1.0;
    //marker.lifetime = ros::Duration(1.0);

    std::random_device seed;
    std::mt19937 engine(seed());            // メルセンヌ・ツイスター法
    rate_.reset( new ros::Rate(30) );
    while(ros::ok()){
        visualization_msgs::MarkerArray marker_array;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(tgt_pt_.x, tgt_pt_.y, 0.0) );
        tf::Quaternion q;
        q.setRPY(0, 0, tgt_theta_);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "true_value" ));

        geometry_msgs::Point position, observed_value, observed_value_add;
        position.x = tgt_pt_.x;
        position.y = tgt_pt_.y;
        position.z = 0.1;

        trajectory.points.push_back( position );
        if ( trajectory.points.size() > 200 ) trajectory.points.erase(trajectory.points.begin());
        trajectory.header.stamp = observed.header.stamp = ros::Time::now();
        pub_true_value_.publish ( position );
        observed_value.x = position.x + (*dist_)(engine);
        observed_value.y = position.y + (*dist_)(engine);
        observed.pose.position = observed_value;
        marker_array.markers.push_back( trajectory );
        marker_array.markers.push_back( observed );

        pub_marker_.publish ( marker_array );
        pub_observed_value_.publish ( observed_value );
        ros::spinOnce();
        rate_->sleep();
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "virtual_environment");
    kalman_filter_tutorial_ros::VirtualEnvironment ve;
    ve.pubData();
    ros::spin();
}