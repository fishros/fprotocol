#ifndef FPROTOCOL_HPP
#define FPROTOCOL_HPP

#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <vector>

#define RING_BUFFER_SIZE 1400
#define FPROTOCOL_FRAME_DATA_SIZE 1400
#define MAX_NODE 10
#define BROADCAST_NODE_ID 0xFF

namespace FProtocol {

// Forward declarations
class Handler;
class RingBuffer;
class Frame;

// Type definitions
using CallbackFunction = std::function<int16_t(uint16_t type, uint8_t from, uint16_t error_code)>;
using ReadFunction = std::function<int32_t(int16_t, uint8_t*, int32_t)>;
using WriteFunction = std::function<int32_t(int16_t, uint8_t*, int32_t)>;
using GetIndexInfoFunction = std::function<class ProtocolData*(uint16_t index)>;
using HeartPingCallback = std::function<int8_t(uint8_t node)>;

// Enums
enum class FieldType : uint8_t {
    UINT8,
    UINT16,
    UINT32,
    INT8,
    INT16,
    INT32,
    FLOAT,
    STRUCT
};

enum class ProtocolType : uint8_t {
    SERVICE_REQUEST_WRITE = 0x01,
    SERVICE_REQUEST_READ,
    SERVICE_RESPONSE_WRITE,
    SERVICE_RESPONSE_READ,
    SERVICE_RESPONSE_ERROR,
    TRANSPORT_DATA,
    HEART_PING,
    HEART_PONG,
    MAX
};

// Structures
struct FieldDescriptor {
    uint16_t offset;
    FieldType type;
    int8_t size_field;
    uint16_t array_len;
    
    FieldDescriptor(uint16_t off, FieldType t, int8_t sf = -1, uint16_t al = 0)
        : offset(off), type(t), size_field(sf), array_len(al) {}
};

struct StructDescriptor {
    uint16_t size;
    uint8_t field_count;
    std::vector<FieldDescriptor> fields;
    
    StructDescriptor(uint16_t s, const std::vector<FieldDescriptor>& f)
        : size(s), field_count(f.size()), fields(f) {}
};

#pragma pack(push, 1)
struct ProtocolHeader {
    uint8_t from;
    uint8_t to;
    uint8_t type;
    uint16_t index;
    uint16_t data_size;
};
#pragma pack(pop)

// Classes
class ProtocolData {
public:
    ProtocolData(uint16_t idx, uint16_t size, void* data_ptr, 
                 CallbackFunction cb = nullptr, 
                 std::shared_ptr<StructDescriptor> desc = nullptr);
    
    uint16_t getIndex() const { return index_; }
    uint16_t getDataSize() const { return data_size_; }
    void* getData() const { return data_; }
    CallbackFunction getCallback() const { return callback_; }
    std::shared_ptr<StructDescriptor> getStructDescriptor() const { return struct_desc_; }
    
    void setCallback(CallbackFunction cb) { callback_ = cb; }
    void setStructDescriptor(std::shared_ptr<StructDescriptor> desc) { struct_desc_ = desc; }

private:
    uint16_t index_;
    uint16_t data_size_;
    void* data_;
    CallbackFunction callback_;
    std::shared_ptr<StructDescriptor> struct_desc_;
};

class RingBuffer {
public:
    explicit RingBuffer(uint32_t size);
    ~RingBuffer();
    
    bool put(const uint8_t* buf, uint32_t size);
    uint32_t get(uint8_t* buf, uint32_t size);
    uint32_t size() const;
    void clear();
    
private:
    std::unique_ptr<uint8_t[]> buf_;
    uint32_t buf_size_;
    uint32_t p_in_;
    uint32_t p_out_;
};

class Frame {
public:
    Frame();
    ~Frame() = default;
    
    void reset();
    
    // Getters
    const ProtocolHeader& getHeader() const { return header_; }
    uint16_t getDataSize() const { return data_size_; }
    uint16_t getRecvSize() const { return recv_size_; }
    uint8_t getFrom() const { return from_; }
    uint16_t getErrorCode() const { return error_code_; }
    ProtocolData* getProtocolData() const { return protocol_data_; }
    uint8_t* getData() const { return data_.get(); }
    
    // Setters
    void setHeader(const ProtocolHeader& header) { header_ = header; }
    void setDataSize(uint16_t size) { data_size_ = size; }
    void setRecvSize(uint16_t size) { recv_size_ = size; }
    void addRecvSize(uint16_t size) { recv_size_ += size; }
    void setFrom(uint8_t from) { from_ = from; }
    void setErrorCode(uint16_t code) { error_code_ = code; }
    void setProtocolData(ProtocolData* data) { protocol_data_ = data; }

private:
    ProtocolHeader header_;
    uint16_t data_size_;
    uint16_t recv_size_;
    uint8_t from_;
    uint16_t error_code_;
    ProtocolData* protocol_data_;
    std::unique_ptr<uint8_t[]> data_;
};

class Handler {
public:
    Handler(ReadFunction read_func, WriteFunction write_func);
    template <typename T, typename ReadMethod, typename WriteMethod>
    Handler(T* obj, ReadMethod read_method, WriteMethod write_method)
        : Handler(
              std::bind(read_method, obj, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
              std::bind(write_method, obj, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)) {}
    ~Handler() = default;
    
    // Configuration
    void setSelfNode(uint8_t node, GetIndexInfoFunction get_index_info);
    void addOtherNode(uint8_t node, GetIndexInfoFunction get_index_info);
    void setHeartPingCallback(HeartPingCallback callback);
    
    // Main operations
    void tick();
    void readPut(const uint8_t* buf, uint32_t size);
    uint16_t write(uint8_t to, ProtocolType type, uint16_t index,
                   const void* data, uint16_t size,
                   std::shared_ptr<StructDescriptor> desc = nullptr);
    
    // Utility functions
    int8_t heartPing(uint8_t target_node);
    ProtocolData* getOtherNodeData(uint8_t node, uint16_t index);
    
    // Static utility functions
    static uint16_t checksum16(const uint8_t* data, size_t length);
    static size_t packStruct(uint8_t* buffer, const void* data, 
                           const StructDescriptor& desc);
    static size_t unpackStruct(const uint8_t* buffer, void* data, 
                             const StructDescriptor& desc);

private:
    void processRequest();
    
    uint8_t self_node_id_;
    GetIndexInfoFunction get_node_info_;
    ReadFunction read_func_;
    WriteFunction write_func_;
    
    std::unique_ptr<Frame> frame_;
    std::unique_ptr<RingBuffer> rx_buffer_;
    
    uint8_t other_node_count_;
    std::vector<uint8_t> other_node_table_;
    std::vector<GetIndexInfoFunction> other_node_info_table_;
    
    HeartPingCallback heart_ping_callback_;
    
    static const uint8_t FRAME_HEAD[4];
};

} // namespace FProtocol

#endif // FPROTOCOL_HPP
