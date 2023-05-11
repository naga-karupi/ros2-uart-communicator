#include <rclcpp/rclcpp.hpp> 
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <chrono>
#include <map>
#include <array>

#include "map_data.hpp"

using namespace std::chrono_literals;

class UARTNode : public rclcpp::Node {
public:
	UARTNode() : Node("uart_node") {
		this->sub      = this->create_subscription<std_msgs::msg::UInt8MultiArray>("uart_msg", 10, std::bind(&UARTNode::sub_callback, this, std::placeholders::_1));
		this->recv_tim = this->create_wall_timer(100ms, std::bind(&UARTNode::recv_timer_callback, this));
		this->pub      = this->create_publisher<std_msgs::msg::UInt8MultiArray>("uart_rx_msg",10);
		this->uart_fail_pub   = this->create_publisher<std_msgs::msg::Bool>("uart_fail_msg", 10);
		this->reopen_port_sub = this->create_subscription<std_msgs::msg::Bool>("uart_reopen", 10, std::bind(&UARTNode::reopen_sub_callback, this, std::placeholders::_1));

		if (!open_serial_port()) {
			uart_fail_publish();
		}
	}

	~UARTNode()	{
		// シリアルポートをクローズ
		if (fd_ >= 0)
		{
			close(fd_);
		}
	}

	bool open_serial_port() {
		// シリアルポートのデバイスファイルを設定
		auto param_serial_name = rcl_interfaces::msg::ParameterDescriptor{};
		auto param_baudrate    = rcl_interfaces::msg::ParameterDescriptor{};
		param_serial_name.description = "";
		this->declare_parameter("serial_name", "", param_serial_name);
		this->declare_parameter("baudrate", 115200, param_baudrate);

		auto device = get_parameter("serial_name").as_string();
		auto baudrate = get_parameter("baudrate").as_int();

		RCLCPP_INFO(this->get_logger(), "device  : %s", device.c_str());
		RCLCPP_INFO(this->get_logger(), "baudrate: %d", baudrate);

		bool is_same_rate = false;
		for(const auto& rate: baudrate_list) {
			if (baudrate == rate) {
				is_same_rate = true;
				break;
			}
		}

		if(!is_same_rate) {
			RCLCPP_ERROR(this->get_logger(), "不正なbaudrateです");
			return false;
		}

		// シリアルポートをオープン
		fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
		if (fd_ < 0) {
			RCLCPP_ERROR(this->get_logger(), "シリアルポートを開けませんでした");
			return false;
		}

		// シリアル通信の設定
		struct termios serial;
		tcgetattr(fd_, &serial);
		if (-1 == tcgetattr(fd_, &serial)) {
			close(fd_);
			std::cout << "tcgetattr error!" << std::endl;
			return false;
		}
		int UART_BAUDRATE = baudrate_map.at(baudrate);
		struct termios term;

		if (-1 == tcgetattr(fd_, &term)) {
			close(fd_);
			std::cout << "tcsetattr error!" << std::endl;
			return false;
		}

		cfsetospeed(&term, UART_BAUDRATE);
		cfsetispeed(&term, UART_BAUDRATE);
		term.c_cflag &= ~CSIZE;
		term.c_cflag |= CS8; // データビット8
		term.c_cflag &= ~CRTSCTS; // ハードウェアフロー制御なし
		term.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
		term.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR);
		term.c_oflag &= ~OPOST;
		term.c_cc[VMIN] = 0;		//	ノンブロッキング
		term.c_cc[VTIME]= 0;

		if (-1 == tcsetattr(fd_, TCSANOW, &term)) {
			close(fd_);
			std::cout << "tcsetattr error!" << std::endl;
			return false;
		}
		return true;
	}

	void send(const std::string &data) {
		if(write(fd_, data.c_str(), data.length()) == -1) {
			RCLCPP_ERROR(this->get_logger(), "送信に失敗しました");
			uart_fail_publish();
		} else {}
	}
  
	void sub_callback(std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
		if(fd_ < 0) {
			RCLCPP_ERROR(this->get_logger(), "シリアルポートが開いていません");
			uart_fail_publish();
			return ;
		}

		std::string s;
		s.resize(msg->data.size());
		for(unsigned long i = 0; i < msg->data.size(); i++) {
			s[i] = msg->data[i];
		}

		/// for show sending msg
//		for(int i = 0; i < s.size(); i++) std::cout << (int)s.at(i) << ", "; std::cout << std::endl;

		send(s);
	}
	
	void reopen_sub_callback(std_msgs::msg::Bool::SharedPtr msg) {
		if(msg->data) {
			if(fd_ >= 0) {
				close(fd_);
			}
			if(!open_serial_port()) {
				uart_fail_publish();
			}
		}
	}

	void recv_timer_callback() {
		if(fd_ < 0) {
			RCLCPP_ERROR(this->get_logger(), "シリアルポートが開いていません");
			uart_fail_publish();
			return ;
		}
		
		char buff[256];

		int n = read(fd_, buff, 256);
		if(n > 0) {
			std::string recv_data(buff, n);
			RCLCPP_INFO(this->get_logger(), "recv %s", recv_data.c_str());

			std_msgs::msg::UInt8MultiArray pub_msg;
			pub_msg.data.resize(n);
			for (unsigned long i = 0; i < pub_msg.data.size(); i++) {
				pub_msg.data[i] = buff[i];
			}
		}
	}
	
	void uart_fail_publish() {
		std_msgs::msg::Bool msg;
		msg.data = true;
		uart_fail_pub->publish(msg);
	}

private:
	int fd_;                       // シリアルポートのファイルディスクリプタ
	std::thread receive_thread_;   // 受信
	rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub;
	rclcpp::TimerBase::SharedPtr recv_tim;
	rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr uart_fail_pub;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reopen_port_sub;
};

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<UARTNode>());
	rclcpp::shutdown();

	return 0;
}