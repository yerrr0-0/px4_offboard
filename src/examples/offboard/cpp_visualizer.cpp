#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

geometry_msgs::msg::PoseStamped vector2PoseMsg(std::string frame_id, std::array<double, 3> position, std::array<double, 4> attitude)
{
    geometry_msgs::msg::PoseStamped pose_msg{};
    pose_msg.header.frame_id = frame_id;
    pose_msg.pose.orientation.w = attitude[0];
    pose_msg.pose.orientation.x = attitude[1];
    pose_msg.pose.orientation.y = attitude[2];
    pose_msg.pose.orientation.z = attitude[3];
    pose_msg.pose.position.x = position[0];
    pose_msg.pose.position.y = position[1];
    pose_msg.pose.position.z = position[2];
    return pose_msg;
}

class CppVisualizer : public rclcpp::Node
{
public:
	CppVisualizer () : Node("cpp_visualizer"), timer_period(0.05)
	{
        vehicle_attitude = {1.0, 0.0, 0.0, 0.0};
        vehicle_local_position = {0.0, 0.0, 0.0};
        vehicle_local_velocity  = {0.0, 0.0, 0.0};
        setpoint_position = {0.0, 0.0, 0.0};

        vehicle_path_msg = nav_msgs::msg::Path();
        setpoint_path_msg = nav_msgs::msg::Path();

        const auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();

        auto vehicle_attitude_callback = [this](VehicleAttitude::ConstSharedPtr msg) -> void{
            vehicle_attitude[0] = msg->q[0];
            vehicle_attitude[1] = msg->q[1];
            vehicle_attitude[2] = -msg->q[2];
            vehicle_attitude[3] = -msg->q[3];                          
        };

        auto vehicle_local_position_callback = [this](VehicleLocalPosition::ConstSharedPtr msg) -> void{
            vehicle_local_position[0] = msg->x;
            vehicle_local_position[1] = -msg->y;
            vehicle_local_position[2] = -msg->z;
            vehicle_local_velocity[0] = msg->vx;
            vehicle_local_velocity[1] = -msg->vy;
            vehicle_local_velocity[2] = -msg->vz;
        };

        auto trajectory_setpoint_callback = [this](TrajectorySetpoint::ConstSharedPtr msg) -> void{
            setpoint_position[0] = msg->position[0];
            setpoint_position[1] = -msg->position[1];
            setpoint_position[2] = -msg->position[2];
        };        
        
        attitude_sub_ = this->create_subscription<VehicleAttitude>(
            "/px4_1/fmu/out/vehicle_attitude", 
            qos_profile, 
            vehicle_attitude_callback);
        local_position_sub_ = this->create_subscription<VehicleLocalPosition>(
            "/px4_1/fmu/out/vehicle_local_position",
            qos_profile, 
            vehicle_local_position_callback);
        setpoint_sub_ = this->create_subscription<TrajectorySetpoint>(
            "/px4_1/fmu/in/trajectory_setpoint", 
            qos_profile, 
            trajectory_setpoint_callback);

        vehicle_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/px4_visualizer/vehicle_pose",
            qos_profile);
        vehicle_vel_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/px4_visualizer/vehicle_velocity",
            qos_profile);
        vehicle_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/px4_visualizer/vehicle_path", 
            qos_profile);
        setpoint_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/px4_visualizer/setpoint_path", 
            qos_profile);
        

        timer_ = this->create_wall_timer(100ms, std::bind(&CppVisualizer::cmdloop_callback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_sub_;
    rclcpp::Subscription<TrajectorySetpoint>::SharedPtr setpoint_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vehicle_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vehicle_path_pub_, setpoint_path_pub_;
    
    std::array<double, 4> vehicle_attitude;
    std::array<double, 3> vehicle_local_position, vehicle_local_velocity, setpoint_position;
    double timer_period;
    nav_msgs::msg::Path vehicle_path_msg;
    nav_msgs::msg::Path setpoint_path_msg;    
    void cmdloop_callback();
    visualization_msgs::msg::Marker create_arrow_marker(int, std::array<double, 3>, std::array<double, 3>);
};

visualization_msgs::msg::Marker CppVisualizer::create_arrow_marker(int id, std::array<double, 3> tail, std::array<double, 3> vector)
{
    visualization_msgs::msg::Marker msg{};
    msg.action = msg.ADD;
    msg.header.frame_id = "map";
    msg.id = id;
    msg.type = msg.ARROW;
    msg.scale.x = 0.1;
    msg.scale.y = 0.2;
    msg.scale.z = 0.0;
    msg.color.r = 0.5;
    msg.color.g = 0.5;
    msg.color.b = 0.0;
    msg.color.a = 1.0;
    float dt = 0.3;
    geometry_msgs::msg::Point tail_point{};
    tail_point.x = tail[0];
    tail_point.y = tail[1];
    tail_point.z = tail[2];
    geometry_msgs::msg::Point head_point{};
    head_point.x = tail[0] + dt * vector[0];
    head_point.y = tail[1] + dt * vector[1];
    head_point.z = tail[2] + dt * vector[2];

    std::vector<geometry_msgs::msg::Point> v_points;
    v_points.reserve(2);
    v_points.emplace_back(tail_point);
    v_points.emplace_back(head_point);  
    msg.points = v_points;
    return msg;
}

void CppVisualizer::cmdloop_callback()
{
    geometry_msgs::msg::PoseStamped vehicle_pose_msg = vector2PoseMsg("map", vehicle_local_position, vehicle_attitude);
    vehicle_pose_pub_->publish(vehicle_pose_msg);

    // Publish time history of the vehicle path
    vehicle_path_msg.header = vehicle_pose_msg.header;
    vehicle_path_msg.poses.emplace_back(vehicle_pose_msg);
    // vehicle_path_msg.poses.push_back(vehicle_pose_msg);
    vehicle_path_pub_->publish(vehicle_path_msg);

    geometry_msgs::msg::PoseStamped setpoint_pose_msg = vector2PoseMsg("map", setpoint_position, vehicle_attitude);
    setpoint_path_msg.header = setpoint_pose_msg.header;
    setpoint_path_msg.poses.emplace_back(setpoint_pose_msg);
    // setpoint_path_msg.poses.push_back(setpoint_pose_msg);
    setpoint_path_pub_->publish(setpoint_path_msg);

    // Publish arrow markers for velocity
    visualization_msgs::msg::Marker velocity_msg = create_arrow_marker(1, vehicle_local_position, vehicle_local_velocity);
    vehicle_vel_pub_->publish(velocity_msg);    
}


int main(int argc, char *argv[])
{
	std::cout << "cpp_visualizer node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CppVisualizer>());
	rclcpp::shutdown();
	return 0;
}
