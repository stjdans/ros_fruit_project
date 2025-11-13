#include <memory>
#include <string>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

namespace gazebo
{
class ScalePlugin : public ModelPlugin
{
public:
  ScalePlugin() : ModelPlugin() {}

  void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override
  {
    model_ = _model;
    world_ = model_->GetWorld();

    // ROS2 초기화 (싱글 노드)
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("scale_plugin_node");
    pub_ = node_->create_publisher<std_msgs::msg::Float32>("/scale/weight", 10);

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ScalePlugin::OnUpdate, this));
    
    RCLCPP_INFO(node_->get_logger(), "Scale Plugin loaded successfully.");
  }

private:
  void OnUpdate()
  {
    // 저울 위에 있는 모든 링크 무게 합산
    double total_mass = 0.0;
    for (auto link : model_->GetLinks())
    {
      // 링크가 저울 위에 있는지 Z 좌표로 단순 체크
      ignition::math::Vector3d pos = link->WorldPose().Pos();
      if (pos.Z() <= 0.6)  // 저울 높이 기준, 필요에 맞게 조정
      {
        total_mass += link->GetInertial()->Mass();
      }
    }

    // ROS2로 publish
    auto msg = std_msgs::msg::Float32();
    msg.data = total_mass;
    pub_->publish(msg);
  }

private:
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  event::ConnectionPtr update_connection_;

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
};

// Gazebo 플러그인 등록
GZ_REGISTER_MODEL_PLUGIN(ScalePlugin)
}  // namespace gazebo
