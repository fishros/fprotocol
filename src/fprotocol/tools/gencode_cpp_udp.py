import os
import sys
from pathlib import Path

from .gencode_cpp import generate_c_code as generate_cpp_base


def write_file(path: Path, content: str):
    path.write_text(content, encoding="utf-8")


def generate_c_code(input_file, output_directory, code_type="cpp"):
    """先用 gencode_cpp 生成协议代码，再追加 UDP 示例 main 和 CMakeLists。"""
    if code_type != "cpp":
        raise ValueError("cpp_udp 仅支持 cpp 输出")

    out_dir = Path(output_directory)
    if output_directory == ".":
        out_dir = Path.cwd()

    # 先生成 Proto.hpp/Proto.cpp 以及 fprotocol.* 拷贝
    generate_cpp_base(input_file, str(out_dir), "cpp")

    file_name = os.path.basename(input_file).split(".")[0]
    file_name = file_name[0].upper() + file_name[1:] if file_name else file_name

    proto_include = f"#include \"{file_name}Proto.hpp\""

    # 构建 data 回调绑定代码：只有存在 *_callback_ 成员才插入
    # 简单做法：检查生成的 Proto.hpp 是否包含 CallbackFunction 成员
    proto_h_path = out_dir / f"{file_name}Proto.hpp"
    has_data_callback = False
    if proto_h_path.exists():
        proto_h_text = proto_h_path.read_text(encoding="utf-8", errors="ignore")
        has_data_callback = "CallbackFunction" in proto_h_text

    data_callback_bind = ""
    data_callback_impl = ""
    if has_data_callback:
        data_callback_bind = (
            "        protocol_.set_data_callback(std::bind(&UdpApp::on_data, this,\n"
            "                                              std::placeholders::_1,\n"
            "                                              std::placeholders::_2,\n"
            "                                              std::placeholders::_3));\n"
        )
        data_callback_impl = (
            "    int16_t on_data(uint16_t type, uint8_t from, uint16_t error_code) {\n"
            "        std::printf(\"cb data: type=0x%02X from=0x%02X err=0x%04X\\n\", type, from, error_code);\n"
            "        return 0;\n"
            "    }\n"
        )

    test_main = f'''#include "fprotocol.hpp"
{proto_include}
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

class UdpApp {{
public:
    UdpApp(uint16_t listen_port, uint8_t self_node_id,
           const char* remote_ip = nullptr, uint16_t remote_port = 0)
        : listen_port_(listen_port), self_node_id_(self_node_id) {{
        if (!init_udp(listen_port_)) {{ ok_ = false; return; }}
        if (remote_ip && std::strlen(remote_ip) > 0) {{
            if (remote_port == 0) remote_port = listen_port_;
            set_remote(remote_ip, remote_port);
        }}
        handler_ = std::make_unique<Handler>(this, &UdpApp::udp_read_impl, &UdpApp::udp_write_impl);
        handler_->setSelfNode(self_node_id_, protocol_.getIndexInfoFunction());
        handler_->setHeartPingCallback(std::bind(&UdpApp::on_heart, this, std::placeholders::_1));
{data_callback_bind}        ok_ = true;
        std::printf("Listening UDP on port %u\\n", listen_port_);
    }}

    bool ok() const {{ return ok_; }}
    void run_forever(int tick_hz = 200) {{
        if (!ok_) return;
        running_ = true;
        const int sleep_us = tick_hz > 0 ? (1000000 / tick_hz) : 0;
        while (running_) {{
            handler_->tick();
            if (sleep_us > 0) std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
        }}
    }}
    void stop() {{ running_ = false; }}

private:
    static void close_socket(socket_t s) {{ if (s != kInvalidSocket) close(s); }}
    static bool set_nonblocking(socket_t s) {{
        int flags = fcntl(s, F_GETFL, 0);
        if (flags < 0) return false;
        return fcntl(s, F_SETFL, flags | O_NONBLOCK) == 0;
    }}
    static bool parse_ipv4(const char* ip, in_addr* out) {{ return inet_pton(AF_INET, ip, out) == 1; }}

    bool init_udp(uint16_t listen_port) {{
        udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_ == kInvalidSocket) return false;
        int reuse = 1;
        setsockopt(udp_socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        sockaddr_in addr{{}};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(listen_port);
        if (bind(udp_socket_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) return false;
        if (!set_nonblocking(udp_socket_)) return false;
        return true;
    }}

    void set_remote(const char* ip, uint16_t port) {{
        sockaddr_in addr{{}};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        if (!parse_ipv4(ip, &addr.sin_addr)) {{ has_remote_ = false; return; }}
        remote_addr_ = addr; has_remote_ = true;
    }}

    int32_t udp_read_impl(int16_t, uint8_t* buf, int32_t size) {{
        if (udp_socket_ == kInvalidSocket) return 0;
        sockaddr_in sender{{}}; socklen_t sender_len = sizeof(sender);
        int received = recvfrom(udp_socket_, buf, size, 0,
                                reinterpret_cast<sockaddr*>(&sender), &sender_len);
        if (received < 0) {{
            if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
            std::perror("recvfrom");
            return 0;
        }}
        last_sender_ = sender; has_sender_ = true; return received;
    }}

    int32_t udp_write_impl(int16_t, uint8_t* buf, int32_t size) {{
        if (udp_socket_ == kInvalidSocket) return 0;
        const sockaddr_in* dest = nullptr;
        if (has_remote_) dest = &remote_addr_;
        else if (has_sender_) dest = &last_sender_;
        else return 0;
        int sent = sendto(udp_socket_, buf, size, 0,
                          reinterpret_cast<const sockaddr*>(dest), sizeof(*dest));
        if (sent < 0) {{ std::perror("sendto"); return 0; }}
        return sent;
    }}

    int8_t on_heart(uint8_t from) {{
        std::printf("heart_cb: from=0x%02X\\n", from);
        return 0;
    }}
{data_callback_impl}
private:
    bool ok_ = false;
    bool running_ = false;
    uint16_t listen_port_ = 0;
    uint8_t self_node_id_ = 0x01;
    std::unique_ptr<Handler> handler_;
    {file_name}Protocol protocol_;
    socket_t udp_socket_ = kInvalidSocket;
    sockaddr_in last_sender_{{}};
    bool has_sender_ = false;
    sockaddr_in remote_addr_{{}};
    bool has_remote_ = false;
}};

int main() {{
    const uint16_t listen_port = 8888;
    const char* remote_ip = nullptr;
    const uint16_t remote_port = 0;
    const uint8_t self_node_id = 0x02;
    UdpApp app(listen_port, self_node_id, remote_ip, remote_port);
    if (!app.ok()) return 1;
    app.run_forever(200);
    return 0;
}}
'''

    cmake = f'''cmake_minimum_required(VERSION 3.10)
project(FProtocolUdpExample)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(fprotocol_cpp fprotocol.cpp)
target_include_directories(fprotocol_cpp PUBLIC ${{CMAKE_CURRENT_SOURCE_DIR}})

add_library(proto {file_name}Proto.cpp)
target_link_libraries(proto fprotocol_cpp)
target_include_directories(proto PUBLIC ${{CMAKE_CURRENT_SOURCE_DIR}})

add_executable(main {file_name}Main.cpp)
target_link_libraries(main proto fprotocol_cpp)
'''

    write_file(out_dir / f"{file_name}Main.cpp", test_main)
    write_file(out_dir / "CMakeLists.txt", cmake)


def main():
    if len(sys.argv) != 3:
        print("Usage: python gencode_cpp_udp.py <input_file> <output_directory>")
        sys.exit(1)
    generate_c_code(sys.argv[1], sys.argv[2], "cpp")


if __name__ == "__main__":
    main()
