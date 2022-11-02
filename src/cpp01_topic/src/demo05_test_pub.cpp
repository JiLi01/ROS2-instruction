/*  
  需求：以某个固定频率发送文本学生信息，包含学生的姓名、年龄、身高等数据。
*/

// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"

using namespace std::chrono_literals;
using base_interfaces_demo::msg::Student;
// 3.定义节点类；
class TalkerStu : public rclcpp::Node
{
  public:
    TalkerStu()
    : Node("talker_node_cpp",rclcpp::NodeOptions().allow_undeclared_parameters(true)),  
    count_(0)
    {
      this->declare_parameter("height",1.5);
      // 3-1.创建发布方；
      publisher_ = this->create_publisher<Student>("chatter_stu", 10);
      // 3-2.创建定时器；
      timer_ = this->create_wall_timer(500ms, std::bind(&TalkerStu::on_timer, this));
    }

  private:
    void on_timer()
    {
      // 3-3.组织消息并发布。
      auto stu = Student();
      stu.name = "hulma";
      stu.age = count_++;
      stu.height = 1.65;
      this->set_parameter(rclcpp::Parameter("height", 1.88));
      auto height_pub = this->get_parameter("height");
      RCLCPP_INFO(this->get_logger(),"身高 = %s", height_pub.value_to_string().c_str());
      RCLCPP_INFO(this->get_logger(), "学生信息:name=%s,age=%d,height=%.2f", stu.name.c_str(),stu.age,stu.height);
      publisher_->publish(stu);

    }

   
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<Student>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  // 2.初始化 ROS2 客户端；
  rclcpp::init(argc, argv);
  // 4.调用spin函数，并传入节点对象指针。
  auto node = std::make_shared<TalkerStu>();
  rclcpp::spin(node);
  // 5.释放资源；
  rclcpp::shutdown();
  return 0;
}