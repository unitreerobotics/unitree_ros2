# 通过ROS2实现与Go2通信

本文档将介绍如何安装、配置ROS-Foxy，实现ROS与Go2之间的通信。

*   由于ROS2-foxy默认cyclonedds的版本为0.7.x，而Go2使用的则是更新的cyclonedds-0.10.2版本，因此本档对如何替换DDS版本进行说明。
    
*   通过例程案例实现在 ROS 2系统中与 Go2 进行通讯。
    

## 系统安装配置

1.首先安装系统环境，版本为Ubuntu20.04LTS。

2. 打开“更换系统源”软件，选择Ubuntu软件->下载自->其他，在弹出的窗口中选择合适的服务器，例如图中的mirrors.aliyun.com。

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/W4j6OJ2awDgbO3p8/img/659d32db-0380-4b2b-94c7-93c838a91e11.png)

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/W4j6OJ2awDgbO3p8/img/2ed45361-be64-45d3-8c19-f46acec36d68.png)

3.安装系统必备的工具

    sudo apt install -y git cmake boost-dev net-tools

## ROS2foxy安装

1.  确定Ubuntu系统的UTF-8支持：打开终端，依次输入下列命令，检查UTF-8设置。
    

    locale  # check for UTF-8
    
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    
    locale  # verify settings

如出现如下内容即为正常：

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/W4j6OJ2awDgbO3p8/img/f9e93462-acc4-485d-a8a9-30626048c6b0.png)

2、 设置 ROS 源，打开终端输入：

    #首先启用 Ubuntu Universe 存储库。
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    
    #添加带有 apt 的 ROS 2 GPG 密钥。
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    #然后将存储库添加到源列表中
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

3、更新apt缓存，可选择新软件包。

    sudo apt update
    sudo apt upgrade #可选

4、安装桌面版ROS-Foxy和其必要组件：

    sudo apt install ros-foxy-desktop python3-argcomplete
    sudo apt install ros-dev-tools

5、至此完成ros-foxy的安装。打开一个新的终端，并输入如下命令，

    source /opt/ros/foxy/setup.bash
    ros2 topic list

如出以下画面即为成功。

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/W4j6OJ2awDgbO3p8/img/334f2895-e060-4013-b11f-8780bd72a4aa.png)

## ROS2 修改cyclonedds版本

由于 ROS 2 foxy 默认 cyclonedds 的版本为 0.7.x ，而 Go2 使用的则是更新的 cyclonedds 0.10.2 版本，如果直接进行通讯，则会出现“segmentation fault”的错误，因此还需要替换 ROS 2 系统自带的 cyclone-dds。替换过程如下：

1、首先安装rmw\_cyclonedds\_cpp功能包

    sudo apt install ros-foxy-rmw-cyclonedds-cpp

2、新建 ROS 2 的工作区

    mkdir -p ~/cyclonedds_ws/src

3、下载对应版本的 cyclone-dds

    cd ~/cyclonedds_ws/src
    git clone https://github.com/ros2/rmw_cyclonedds -b foxy
    git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x

4、下载数据接口文件

注： 后续 gitlab 的 SDK 要放到 git 上。再替换该内容和说明

    cd ~/cyclonedds_ws/src
    git clone http://192.168.1.102/zeende/unitree_go

5、编译程序

    cd ~/cyclonedds_ws/
    source /opt/ros/foxy/setup.bash
    colcon build --symlink-install

6、将 ROS 写入环境变量

    #打开终端 输入下列命令
    gedit ~/.bashrc

在弹出的文件最下面写入如下命令：

*   source /opt/ros/foxy/setup.bash
    
*   source ~/cyclonedds\_ws/install/setup.bash
    
*   export RMW\_IMPLEMENTATION=rmw\_cyclonedds\_cpp
    
*   export CYCLONEDDS\_URI=~/cyclonedds\_ws/src/unitree\_go/cyclonedds.xml
    

保存后重开终端，若输入 ros2 topic list 后未报错，则说明替换成功。

## 测试通信

测试 Go2 与计算机之间的通信主要步骤如下：

1.  使用网线连接 Go2 和计算机，之后打开网络设置，进入 IPv4 ，将 IPv4 方式改为手动，地址设置为192.168.123.99，子网掩码设置为255.255.255.0，完成后点击应用，等待网络重新连接。
    

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/W4j6OJ2awDgbO3p8/img/721e1660-04dc-42b7-8d6e-14799afe2165.png)

2.  打开终端，输入ifconfig命令，记录刚刚设置 IP 后的网卡编号（例如如图中的en3s0，以自己电脑实际为准）
    

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/W4j6OJ2awDgbO3p8/img/5d22c143-5dad-4964-81f3-55864906a9f0.png)

3.  打开~/cyclonedds\_ws/src/unitree\_go/cyclonedds.xml文件，修改如图所示的NetworkInterface标签的name，修改结果如下（具体值以实际为准）。
    

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/W4j6OJ2awDgbO3p8/img/9663ae5e-c58b-4401-b82c-15d0fdb2085c.png)

4.  网络重新连接后，打开终端输入ros2 topic list，可以看见如下话题：
    

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/W4j6OJ2awDgbO3p8/img/5e45e8ec-9248-47eb-8380-798ed0ef468b.png)

5.  打开终端输入ros2 topic echo /sportmodestate 后，可以看见该话题的数据如下图所示，说明通讯正常：
    

![image](https://alidocs.oss-cn-zhangjiakou.aliyuncs.com/res/W4j6OJ2awDgbO3p8/img/89214761-6cfb-4b52-bf24-7a5bd9a9806c.png)

## Example

### 例程测试

该例程实现对 Go2 的移动控制。

1.  首先新建工作空间并将案例程序下载编译，输入如下命令：
    

    #打开终端
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone http://192.168.1.102/zeende/go2_demo
    cd ~/ros2_ws
    colcon build

2.  运行例程控制 Go2 前进，输入如下命令：
    

    #打开终端
    source ~/ros2_ws/install/setup/bash
    ros2 run go2_demo demo

### 例程解析

本节对 demo.cpp 中的程序进行介绍。

    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/string.hpp"
    #include "unitree_go/msg/sport_mode_cmd.hpp"
    
    class Cmd_Sender : public rclcpp::Node
    {
    public:
        // 构造函数,有一个参数为节点名称
        Cmd_Sender() : Node("Cmd_Sender")
        {
            //RCLCPP_INFO(this->get_logger(), "大家好，我是%s.", name.c_str());
            // 创建发布者
            command_publisher_ = this->create_publisher<unitree_go::msg::SportModeCmd>("/sportmodecmd", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Cmd_Sender::timer_callback, this));
        }
    
    private:
        void timer_callback()
        {
            // 创建消息
            unitree_go::msg::SportModeCmd message;
            // 设置模式
            message.mode = 2;
            message.gait_type = 1;
            // 设置角速度
            message.yaw_speed = 0.0;
            // 设置速度，{vx,vy}
            message.velocity = {0.1, 0};
            // 日志打印
            // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            // 发布消息
            command_publisher_->publish(message);
        }
        // 声名定时器指针
        rclcpp::TimerBase::SharedPtr timer_;
        // 声明话题发布者
        rclcpp::Publisher<unitree_go::msg::SportModeCmd>::SharedPtr command_publisher_;
    };
    
    int main(int argc, char **argv)
    {
        rclcpp::init(argc, argv);
        rclcpp::TimerBase::SharedPtr timer_;
        /*创建对应节点的共享指针对象*/
        auto node = std::make_shared<Cmd_Sender>();
        /* 运行节点，并检测退出信号*/
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }

该程序中unitree\_go::msg::SportModeCmd message创建了控制 Go2 的话题数据，在对角速度 message.yaw\_speed和 xy 方向速度message.velocity设置后，发布该控制话题，机器人将按照设置值运行。