#include "fprotocol.hpp"
#include "RobotProto.hpp"
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <memory>
#include <thread>
#include <chrono>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

using namespace FProtocol;

using socket_t = int;
static constexpr socket_t kInvalidSocket = -1;

class UdpApp {
public:
    UdpApp(uint16_t listen_port, uint8_t self_node_id,
           const char* remote_ip = nullptr, uint16_t remote_port = 0)
        : listen_port_(listen_port), self_node_id_(self_node_id) {
        if (!init_udp(listen_port_)) { ok_ = false; return; }
        if (remote_ip && std::strlen(remote_ip) > 0) {
            if (remote_port == 0) remote_port = listen_port_;
            set_remote(remote_ip, remote_port);
        }
        handler_ = std::make_unique<Handler>(this, &UdpApp::udp_read_impl, &UdpApp::udp_write_impl);
        handler_->setSelfNode(self_node_id_, protocol_.getIndexInfoFunction());
        handler_->setHeartPingCallback(std::bind(&UdpApp::on_heart, this, std::placeholders::_1));
        protocol_.set_data_callback(std::bind(&UdpApp::on_data, this,
                                              std::placeholders::_1,
                                              std::placeholders::_2,
                                              std::placeholders::_3));
        ok_ = true;
        std::printf("Listening UDP on port %u\n", listen_port_);
    }

    bool ok() const { return ok_; }
    void run_forever(int tick_hz = 200) {
        if (!ok_) return;
        running_ = true;
        const int sleep_us = tick_hz > 0 ? (1000000 / tick_hz) : 0;
        while (running_) {
            handler_->tick();
            if (sleep_us > 0) std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
        }
    }
    void stop() { running_ = false; }

private:
    static void close_socket(socket_t s) { if (s != kInvalidSocket) close(s); }
    static bool set_nonblocking(socket_t s) {
        int flags = fcntl(s, F_GETFL, 0);
        if (flags < 0) return false;
        return fcntl(s, F_SETFL, flags | O_NONBLOCK) == 0;
    }
    static bool parse_ipv4(const char* ip, in_addr* out) { return inet_pton(AF_INET, ip, out) == 1; }

    bool init_udp(uint16_t listen_port) {
        udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_ == kInvalidSocket) return false;
        int reuse = 1;
        setsockopt(udp_socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(listen_port);
        if (bind(udp_socket_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) return false;
        if (!set_nonblocking(udp_socket_)) return false;
        return true;
    }

    void set_remote(const char* ip, uint16_t port) {
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        if (!parse_ipv4(ip, &addr.sin_addr)) { has_remote_ = false; return; }
        remote_addr_ = addr; has_remote_ = true;
    }

    int32_t udp_read_impl(int16_t, uint8_t* buf, int32_t size) {
        if (udp_socket_ == kInvalidSocket) return 0;
        sockaddr_in sender{}; socklen_t sender_len = sizeof(sender);
        int received = recvfrom(udp_socket_, buf, size, 0,
                                reinterpret_cast<sockaddr*>(&sender), &sender_len);
        if (received < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
            std::perror("recvfrom");
            return 0;
        }
        last_sender_ = sender; has_sender_ = true; return received;
    }

    int32_t udp_write_impl(int16_t, uint8_t* buf, int32_t size) {
        if (udp_socket_ == kInvalidSocket) return 0;
        const sockaddr_in* dest = nullptr;
        if (has_remote_) dest = &remote_addr_;
        else if (has_sender_) dest = &last_sender_;
        else return 0;
        int sent = sendto(udp_socket_, buf, size, 0,
                          reinterpret_cast<const sockaddr*>(dest), sizeof(*dest));
        if (sent < 0) { std::perror("sendto"); return 0; }
        return sent;
    }

    int8_t on_heart(uint8_t from) {
        std::printf("heart_cb: from=0x%02X\n", from);
        return 0;
    }
    int16_t on_data(uint16_t type, uint8_t from, uint16_t error_code) {
        std::printf("cb data: type=0x%02X from=0x%02X err=0x%04X\n", type, from, error_code);
        return 0;
    }

private:
    bool ok_ = false;
    bool running_ = false;
    uint16_t listen_port_ = 0;
    uint8_t self_node_id_ = 0x01;
    std::unique_ptr<Handler> handler_;
    RobotProtocol protocol_;
    socket_t udp_socket_ = kInvalidSocket;
    sockaddr_in last_sender_{};
    bool has_sender_ = false;
    sockaddr_in remote_addr_{};
    bool has_remote_ = false;
};

int main() {
    const uint16_t listen_port = 8889;
    const char* remote_ip = nullptr;
    const uint16_t remote_port = 0;
    const uint8_t self_node_id = 0x01;
    UdpApp app(listen_port, self_node_id, remote_ip, remote_port);
    if (!app.ok()) return 1;
    app.run_forever(200);
    return 0;
}
