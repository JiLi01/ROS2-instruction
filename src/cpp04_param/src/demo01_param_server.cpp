/*
    需求：编写参数服务端，设置并操作参数。
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.声明参数；
            3-2.查询参数；
            3-3.修改参数；
            3-4.删除参数。
        4.创建节点对象指针，调用参数操作函数，并传递给spin函数；
        5.释放资源。

*/

// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"

// 3.定义节点类；
class ParamServer: public rclcpp::Node{
    public:
        ParamServer():Node("param_server_node_cpp",rclcpp::NodeOptions()
                .allow_undeclared_parameters(true)){  
                  RCLCPP_INFO(this->get_logger(),"参数服务端创建了！");       
        }
        // 3-1.声明参数；
        void declare_param(){
            RCLCPP_INFO(this->get_logger(),"------------------增----------------");
            // 声明参数并设置默认值
            this->declare_parameter("car_name","Tiger"); 
            this->declare_parameter("width",1.50); 
            this->declare_parameter("wheels",4);   
            // 需要设置 rclcpp::NodeOptions().allow_undeclared_parameters(true),否则非法 
            this->set_parameter(rclcpp::Parameter("height",1.75));
        }
        // 3-2.查询参数
        void get_param(){
            RCLCPP_INFO(this->get_logger(),"------------------查----------------");
            // 获取指定
            auto car = this->get_parameter("car_name");
            RCLCPP_INFO(this->get_logger(),"key = %s, value = %s",car.get_name().c_str(),car.as_string().c_str());
            
            // 获取一些参数
            auto params = this->get_parameters({"car_name","width","wheels"});
            for (auto &&param : params)
            {
                RCLCPP_INFO(this->get_logger(),"%s = %s", param.get_name().c_str(), param.value_to_string().c_str());

            }

            // 判断包含
            RCLCPP_INFO(this->get_logger(),"包含car_name? %d",this->has_parameter("car_name"));
            RCLCPP_INFO(this->get_logger(),"包含car_namesxxxx? %d",this->has_parameter("car_namexxxx"));
            
        }
        // 3-3.修改参数
        void update_param(){
            RCLCPP_INFO(this->get_logger(),"------------------改----------------");
            this->set_parameter(rclcpp::Parameter("width",1.75));
            RCLCPP_INFO(this->get_logger(),"width:%.2f",this->get_parameter("width").as_double());
        }
        // 3-4.删除参数
        void del_param(){
            RCLCPP_INFO(this->get_logger(),"------------------删----------------");
            // this->undeclare_parameter("car_name");
            // RCLCPP_INFO(this->get_logger(),"删除操作后，car_name还存在马? %d",this->has_parameter("car_name"));
            RCLCPP_INFO(this->get_logger(),"删除操作前，height存在马? %d",this->has_parameter("height"));
            this->undeclare_parameter("height");
            RCLCPP_INFO(this->get_logger(),"删除操作前，height存在马? %d",this->has_parameter("height"));
        }
};

int main(int argc, char ** argv)
{
    // 2.初始化 ROS2 客户端；
    rclcpp::init(argc,argv);

    // 4.创建节点对象指针，调用参数操作函数，并传递给spin函数；
    auto node= std::make_shared<ParamServer>();
    node->declare_param();
    node->get_param();
    node->update_param();
    node->del_param();
    rclcpp::spin(node);

    // 5.释放资源。
    rclcpp::shutdown();
    return 0;
}