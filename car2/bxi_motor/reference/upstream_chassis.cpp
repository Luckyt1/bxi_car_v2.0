#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "chassis/modbus4lift.hpp"
#include <cmath>
#include <time.h>
#include <unistd.h>
#include <iostream>
#include "bxi_pci_drv.h"

class Chassis : public rclcpp::Node
{
public:
    Chassis() : Node("chassis"){
        // 初始化
        hardware_init();

        // 订阅
        sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_car", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg)
            {
                last_twist_ = *msg;     
            });

        // lift对象
        lift_dev_ = std::make_shared<Modbus4lift>("/dev/ttyUSB0", 9600, 'N', 8, 1, 1);
        sleep(2);
        // 控制任务
        can_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        modbus_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&Chassis::task, this),modbus_group_);

    }

    ~Chassis() override{
        lift_dev_ = NULL;

        canfd_packet msg[2];

        for(int i = 0; i < 1; ++i){
        msg[i].bus = 2;
        msg[i].frame.can_id = i + 1;
        msg[i].frame.len = 8;
        msg[i].frame.data[0] = 0xFF;
        msg[i].frame.data[1] = 0xFF;
        msg[i].frame.data[2] = 0xFF;
        msg[i].frame.data[3] = 0xFF;
        msg[i].frame.data[4] = 0xFF;
        msg[i].frame.data[5] = 0xFF;
        msg[i].frame.data[6] = 0xFF;
        msg[i].frame.data[7] = 0xFD;
        }
        canfd_send_packet(msg, 2);
        sleep(1);
        motor_pwr_set(0);
        RCLCPP_INFO(get_logger(), "Chassis node 已安全退出");
    }

private:
    void task(){
        RCLCPP_INFO(this->get_logger(),
                    "timer: 线速度=%.2f, 角速度=%.2f",
                    last_twist_.linear.x, last_twist_.angular.z);
        float v[2] = {0};
        diffWheelSpeed(last_twist_.linear.x,last_twist_.angular.z,  0.46, v);

        float p = 0;
        float v_left = -v[0];
        float v_right = v[1]; 
        float kp = 0;
        float kd = 6;
        float t_ff = 0;

        canfd_packet msg[2];
        msg[0].bus = 2;
        msg[1].bus = 2;

        msg[0].frame.can_id = 1;
        msg[1].frame.can_id = 2;

        msg[0].frame.len = 8;
        msg[1].frame.len = 8;

        pack_cmd(msg[0].frame.data, p, v_left, kp, kd, t_ff);
        pack_cmd(msg[1].frame.data, p, v_right, kp, kd, t_ff);

        canfd_send_packet(msg, 2);
        int lift = last_twist_.linear.z;
        if(lift > 0){
            lift_dev_->platformControl(Modbus4lift::PlatformCmd::UP);
        }
        else if(lift < 0){
            lift_dev_->platformControl(Modbus4lift::PlatformCmd::DOWN);
        }
        else{
            lift_dev_->platformControl(Modbus4lift::PlatformCmd::STOP);
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_2;
    rclcpp::CallbackGroup::SharedPtr can_group_;
    rclcpp::CallbackGroup::SharedPtr modbus_group_;
    geometry_msgs::msg::Twist last_twist_; 

    std::shared_ptr<Modbus4lift> lift_dev_;
    
private:
    static int canfd_rx_call_func(void* arg, canfd_packet *msg){
        auto self  = static_cast<Chassis*>(arg);
        self->canfd_rx_call_handle(msg);
        return 0;
    }

    int canfd_rx_call_handle(canfd_packet *msg){
        uint8_t *data = msg->frame.data;

        if(msg->frame.can_id & 0x7F0)
            return 1;

        uint16_t p_int = ((uint16_t)data[1] << 8) | data[2];
        uint16_t v_int = ((uint16_t)data[3] << 4) | (data[4] >> 4);

        float p_out = uint_to_float(p_int, -12.5, 12.5, 16);
        float v_out = uint_to_float(v_int, -45, 45, 12);
        if(msg->frame.can_id == 1){
            RCLCPP_INFO(this->get_logger(),
            "left: 位置=%.2f, 速度=%.2f",
            p_out, v_out);
        }
        else if(msg->frame.can_id == 2){
            RCLCPP_INFO(this->get_logger(),
            "right: 位置=%.2f, 速度=%.2f\n",
            p_out, v_out);
        }
        return 1;
    }
    
private:
    int float_to_uint(float x, float x_min, float x_max, int bits){
        float span = x_max - x_min;
        float offset = x_min;
        return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
    }

    float uint_to_float(int x_int, float x_min, float x_max, int bits){
        float span = x_max - x_min;
        float offset = x_min;
        return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
    }

    void pack_cmd(uint8_t *data, float p_des, float v_des, float kp, float kd, float t_ff){
        uint16_t p_int = float_to_uint(p_des, -12.5f, 12.5f, 16);
        uint16_t v_int = float_to_uint(v_des, -45.0f, 45.0f, 12);
        uint16_t kp_int = float_to_uint(kp, 0.0, 500.0f, 12);
        uint16_t kd_int = float_to_uint(kd, 0.0f, 5.0f, 12);
        uint16_t t_int = float_to_uint(t_ff, -40.0f, 40.0f, 12);
        
        data[0] = p_int >> 8;
        data[1] = p_int & 0xFF;
        data[2] = v_int >> 4;
        data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
        data[4] = kp_int & 0xFF;
        data[5] = kd_int >> 4;
        data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
        data[7] = t_int & 0xff;
    }

    void diffWheelSpeed(float linear_v, float angular_v, float wheel_base, float *wheel_sp){
        float half_base = 0.5f * wheel_base;
        wheel_sp[0] = linear_v - angular_v * half_base;  // 左轮
        wheel_sp[1] = linear_v + angular_v * half_base;  // 右轮
    }

    void hardware_init(){
        bxi_pci_init(Chassis::canfd_rx_call_func, this, -1);
        motor_pwr_set(1);
        sleep(2);
        canfd_packet msg[2];

        for(int i = 0; i < 2; ++i){
            msg[i].bus = 2;
            msg[i].frame.can_id = i + 1;
            msg[i].frame.len = 8;
            msg[i].frame.data[0] = 0xFF;
            msg[i].frame.data[1] = 0xFF;
            msg[i].frame.data[2] = 0xFF;
            msg[i].frame.data[3] = 0xFF;
            msg[i].frame.data[4] = 0xFF;
            msg[i].frame.data[5] = 0xFF;
            msg[i].frame.data[6] = 0xFF;
            msg[i].frame.data[7] = 0xFC;
        }
        canfd_send_packet(msg, 2);
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Chassis>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}