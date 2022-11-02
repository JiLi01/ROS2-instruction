/*
    需求：编写参数客户端，获取或修改服务端参数。
    步骤：
        1.包含头文件；
        2.初始化 ROS2 客户端；
        3.定义节点类；
            3-1.查询参数；
            3-2.修改参数；
        4.创建节点对象指针，调用参数操作函数；
        5.释放资源。
*/

// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

// 3.定义节点类；
class ParamClient: public rclcpp::Node {
    public:
        ParamClient():Node("client_node_cpp"){
            RCLCPP_INFO(this->get_logger(),"参数客户端创建了！");
            param_client_ = std::make_shared<rclcpp::SyncParametersClient>(this,"param_server_node_cpp");
        }
        bool connect_server(){
            // 等待服务连接
            while (!param_client_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                   return false;
                }  
                RCLCPP_INFO(this->get_logger(),"服务连接中...");
            }

            return true;

        }

        // 3-1.查询参数；
        void get_param(){
            RCLCPP_INFO(this->get_logger(),"-----------参数客户端查询参数-----------");
            std::string car_name = param_client_ ->get_parameter<std::string>("car_name");
            double width = param_client_->get_parameter<double>("width");
            RCLCPP_INFO(this->get_logger(),"width = %.2f", width );
            RCLCPP_INFO(this->get_logger(),"car_type 存在吗？%d", param_client_->has_parameter("car_type"));
            auto params = param_client_->get_parameters({"car_name","width","wheels"});
            for (auto &&param : params)
            {
                RCLCPP_INFO(this->get_logger(),"%s = %s", param.get_name().c_str(),param.value_to_string().c_str());
            }


        }
        // 3-2.修改参数；
        void update_param(){
            RCLCPP_INFO(this->get_logger(),"-----------参数客户端修改参数-----------");
            param_client_->set_parameters({rclcpp::Parameter("car_name","Mouse"),
            rclcpp::Parameter("width",2.0),
            //这是服务端不存在的参数，只有服务端设置了rclcpp::NodeOptions().allow_undeclared_parameters(true)时，
            // 这个参数才会被成功设置。
            rclcpp::Parameter("length",5.0),
            rclcpp::Parameter("wheels",6)});
            RCLCPP_INFO(this->get_logger(),"新设置的参数：%.2f",param_client_->get_parameter<double>("length"));
        }

    private:
        rclcpp::SyncParametersClient::SharedPtr param_client_;
};

int main(int argc, char const *argv[])
{
    // 2.初始化 ROS2 客户端；
    rclcpp::init(argc,argv);

    // 4.创建节点对象指针，调用参数操作函数；
    auto client = std::make_shared<ParamClient>();
    bool flag = client->connect_server();
    if(!flag){
        return 0;
    }
    client->get_param();
    //client->update_param();
    client->get_param();
    rclcpp::spin(client);

    // 5.释放资源。
    rclcpp::shutdown();
    return 0;
}