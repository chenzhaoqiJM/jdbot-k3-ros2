#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio.hpp>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <fstream>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <cstring>

std::string get_template_file_path() {
    // 获取当前包的 share 目录路径
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("jobot_websocket_cpp");

    // 拼接 templates/index.html 的路径
    std::string file_path = package_share_directory + "/templates/index.html";

    return file_path;
}


std::string get_host_ip() {
    struct ifaddrs* ifaddr = nullptr;
    struct ifaddrs* ifa = nullptr;
    std::string ip_address = "127.0.0.1"; // 默认回环地址

    try {
        // 获取系统的网络接口地址
        if (getifaddrs(&ifaddr) == -1) {
            perror("getifaddrs");
            return ip_address;
        }

        // 遍历接口列表
        for (ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
            if (ifa->ifa_addr == nullptr) {
                continue;
            }

            // 只处理IPv4地址
            if (ifa->ifa_addr->sa_family == AF_INET) {
                char addr_buffer[INET_ADDRSTRLEN];
                void* addr_ptr = &((struct sockaddr_in*)ifa->ifa_addr)->sin_addr;

                // 转换为可读字符串
                if (inet_ntop(AF_INET, addr_ptr, addr_buffer, sizeof(addr_buffer)) == nullptr) {
                    perror("inet_ntop");
                    continue;
                }

                // 过滤回环地址
                if (std::string(addr_buffer) != "127.0.0.1") {
                    ip_address = addr_buffer;
                    break; // 找到一个非回环地址后直接返回
                }
            }
        }

        freeifaddrs(ifaddr); // 释放内存
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    return ip_address;
}


namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;

class VideoStreamNode : public rclcpp::Node {
public:
    VideoStreamNode()
        : Node("video_stream_node"), acceptor_(ioc_) {

        // Declare and get the topic parameter
        this->declare_parameter<std::string>("image_topic", "/image_raw/compressed");
        std::string image_topic = this->get_parameter("image_topic").as_string();

        this->declare_parameter<int>("port", 8080);
        port_ = this->get_parameter("port").as_int();


        // Initialize ROS subscription
        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            image_topic, 20,
            std::bind(&VideoStreamNode::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "WebSocket Stream Node has started.");

        // Start server in a separate thread
        server_thread_ = std::thread(&VideoStreamNode::run_server, this);
    }

    ~VideoStreamNode() {
        ioc_.stop();
        if (server_thread_.joinable()) {
            server_thread_.join();
        }
    }

    int port_;

private:
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (frame_queue_.size() >= max_queue_size_) {
            frame_queue_.pop(); // Discard oldest frame if the queue is full
        }
        frame_queue_.emplace(msg->data.begin(), msg->data.end());
        queue_cv_.notify_one();
    }

    void run_server() {
        try {
            tcp::endpoint endpoint(tcp::v4(), port_);
            acceptor_.open(endpoint.protocol());
            acceptor_.set_option(tcp::acceptor::reuse_address(true));
            acceptor_.bind(endpoint);
            acceptor_.listen();

            RCLCPP_INFO(this->get_logger(), "Server running on http://0.0.0.0:%d", port_);

            while (rclcpp::ok()) {
                tcp::socket socket(ioc_);
                acceptor_.accept(socket);
                std::thread(&VideoStreamNode::handle_session, this, std::move(socket)).detach();
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Server error: %s", e.what());
        }
    }

    void handle_session(tcp::socket socket) {
        try {
            beast::flat_buffer buffer;
            http::request<http::string_body> req;
            http::read(socket, buffer, req);

            if (websocket::is_upgrade(req)) {
                handle_websocket(std::move(socket), std::move(req));
            } else {
                handle_http(std::move(socket), std::move(req));
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Session error: %s", e.what());
        }
    }

    void handle_http(tcp::socket socket, http::request<http::string_body> req) {
        if (req.method() == http::verb::get && req.target() == "/") {
            std::string html_path = get_template_file_path();
            std::ifstream html_file(html_path);
            if (!html_file) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open index.html");
                return;
            }

            std::string html_content((std::istreambuf_iterator<char>(html_file)), std::istreambuf_iterator<char>());

            // 动态获取主机 IP 地址
            std::string host_ip = get_host_ip();

            std::cout<<"HOST IP IS: "<<host_ip<<std::endl;

            // 替换占位符 {{HOST_IP}} 为实际的 IP 地址
            size_t pos = html_content.find("{{HOST_IP}}");
            if (pos != std::string::npos) {
                html_content.replace(pos, std::string("{{HOST_IP}}").length(), host_ip);
            }

            // 替换 {{PORT}}
            pos = html_content.find("{{PORT}}");
            if (pos != std::string::npos) {
                html_content.replace(pos, std::string("{{PORT}}").length(), std::to_string(port_));
            }

            http::response<http::string_body> res{http::status::ok, req.version()};
            res.set(http::field::content_type, "text/html");
            res.body() = html_content;
            res.prepare_payload();
            http::write(socket, res);
        } else {
            http::response<http::string_body> res{http::status::not_found, req.version()};
            res.set(http::field::content_type, "text/plain");
            res.body() = "404 Not Found";
            res.prepare_payload();
            http::write(socket, res);
        }
    }

    void handle_websocket(tcp::socket socket, http::request<http::string_body> req) {
        try {
            websocket::stream<tcp::socket> ws(std::move(socket));
            ws.accept(req);

            RCLCPP_INFO(this->get_logger(), "WebSocket client connected.");

            while (rclcpp::ok()) {
                std::vector<uint8_t> frame;
                {
                    std::unique_lock<std::mutex> lock(queue_mutex_);
                    queue_cv_.wait(lock, [this]() { return !frame_queue_.empty(); });
                    frame = std::move(frame_queue_.front());
                    frame_queue_.pop();
                }

                std::string encoded_frame = base64_encode(frame.data(), frame.size());
                ws.write(net::buffer(encoded_frame));
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "WebSocket client disconnected: %s", e.what());
        }
    }

    static std::string base64_encode(const uint8_t* bytes, size_t len) {
        // Base64 encoding implementation
        static const std::string chars =
            "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
            "abcdefghijklmnopqrstuvwxyz"
            "0123456789+/";
        std::string encoded;
        int val = 0, valb = -6;
        for (size_t i = 0; i < len; i++) {
            val = (val << 8) + bytes[i];
            valb += 8;
            while (valb >= 0) {
                encoded.push_back(chars[(val >> valb) & 0x3F]);
                valb -= 6;
            }
        }
        if (valb > -6) encoded.push_back(chars[((val << 8) >> (valb + 8)) & 0x3F]);
        while (encoded.size() % 4) encoded.push_back('=');
        return encoded;
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
    std::queue<std::vector<uint8_t>> frame_queue_;
    const size_t max_queue_size_ = 10;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;

    net::io_context ioc_;
    tcp::acceptor acceptor_;
    std::thread server_thread_;

    
};

int main(int argc, char** argv) {
    // 动态获取主机 IP 地址
    std::string host_ip = get_host_ip();
    

    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoStreamNode>();
    std::cout<<"Please visit in your browser: "<< host_ip << ":"<<node->port_<<std::endl;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
