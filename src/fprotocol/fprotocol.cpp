#include "fprotocol.hpp"
#include <algorithm>
#include <iostream>

// #define DEBUG 1

namespace FProtocol {

const uint8_t Handler::FRAME_HEAD[4] = {0x55, 0xAA, 0x55, 0xAA};

// ProtocolData Implementation
ProtocolData::ProtocolData(uint16_t idx, uint16_t size, void* data_ptr, 
                          CallbackFunction cb, 
                          std::shared_ptr<StructDescriptor> desc)
    : index_(idx), data_size_(size), data_(data_ptr), callback_(cb), struct_desc_(desc) {
}

// RingBuffer Implementation
RingBuffer::RingBuffer(uint32_t size) 
    : buf_(std::make_unique<uint8_t[]>(size)), buf_size_(size), p_in_(0), p_out_(0) {
}

RingBuffer::~RingBuffer() = default;

bool RingBuffer::put(const uint8_t* buf, uint32_t size) {
    if (size + this->size() >= buf_size_) {
        return false;
    }
    
    for (uint32_t i = 0; i < size; i++) {
        buf_[p_in_] = buf[i];
        p_in_ = (p_in_ + 1) % buf_size_;
    }
    return true;
}

uint32_t RingBuffer::get(uint8_t* buf, uint32_t size) {
    if (p_in_ == p_out_) {
        return 0;
    }
    if (this->size() < size) {
        return 0;
    }
    
    uint32_t i;
    for (i = 0; i < size; i++) {
        buf[i] = buf_[p_out_];
        p_out_ = (p_out_ + 1) % buf_size_;
        
        if (p_in_ == p_out_) {
            return i + 1;
        }
    }
    return i;
}

uint32_t RingBuffer::size() const {
    if (p_in_ >= p_out_) {
        return p_in_ - p_out_;
    } else {
        return buf_size_ - (p_out_ - p_in_);
    }
}

void RingBuffer::clear() {
    p_in_ = 0;
    p_out_ = 0;
}

// Frame Implementation
Frame::Frame() 
    : data_size_(0), recv_size_(0), from_(0), error_code_(0), 
      protocol_data_(nullptr), data_(std::make_unique<uint8_t[]>(FPROTOCOL_FRAME_DATA_SIZE)) {
    std::memset(&header_, 0, sizeof(header_));
}

void Frame::reset() {
    recv_size_ = 0;
    data_size_ = 0;
    from_ = 0;
    error_code_ = 0;
    protocol_data_ = nullptr;
    std::memset(&header_, 0, sizeof(header_));
}

// Handler Implementation
Handler::Handler(ReadFunction read_func, WriteFunction write_func)
    : self_node_id_(0), read_func_(read_func), write_func_(write_func),
      frame_(std::make_unique<Frame>()), 
      rx_buffer_(std::make_unique<RingBuffer>(RING_BUFFER_SIZE)),
      other_node_count_(0) {
    other_node_table_.reserve(MAX_NODE);
    other_node_info_table_.reserve(MAX_NODE);
}

void Handler::setSelfNode(uint8_t node, GetIndexInfoFunction get_index_info) {
    self_node_id_ = node;
    get_node_info_ = get_index_info;
}

void Handler::addOtherNode(uint8_t node, GetIndexInfoFunction get_index_info) {
    if (other_node_count_ >= MAX_NODE) {
        return;
    }
    other_node_table_.push_back(node);
    other_node_info_table_.push_back(get_index_info);
    other_node_count_++;
}

ProtocolData* Handler::getOtherNodeData(uint8_t node, uint16_t index) {
    if (other_node_count_ == 0) {
        return nullptr;
    }
    
    for (size_t i = 0; i < other_node_count_; i++) {
        if (other_node_table_[i] == node) {
            return other_node_info_table_[i](index);
        }
    }
    return nullptr;
}

void Handler::readPut(const uint8_t* buf, uint32_t size) {
    rx_buffer_->put(buf, size);
#ifdef DEBUG
    std::printf("put data to ring_buffer_size: %d\n", size);
#endif
}

void Handler::tick() {
    static uint8_t data[256];
    static uint8_t from = 0;
    uint8_t rdata;
    
    if (read_func_) {
        int16_t rsize = read_func_(from, data, 256);
        if (rsize > 0) {
            rx_buffer_->put(data, rsize);
#ifdef DEBUG
            std::printf("put data to ring_buffer_size: %d\n", rsize);
#endif
        }
    }

#ifdef DEBUG
    std::printf("ring_size: %d frame->recv_size:%u\n", rx_buffer_->size(), frame_->getRecvSize());
#endif

    if (frame_->getRecvSize() < 4) {
        while (frame_->getRecvSize() < 4) {
            if (rx_buffer_->get(&rdata, 1)) {
#ifdef DEBUG
                std::printf("rdata: %02x\n", rdata);
#endif
                if (rdata == FRAME_HEAD[frame_->getRecvSize()]) {
                    frame_->getData()[frame_->getRecvSize()] = rdata;
                    frame_->addRecvSize(1);
                } else if (rdata == FRAME_HEAD[0]) {
                    frame_->getData()[0] = rdata;
                    frame_->setRecvSize(1);
                } else {
                    frame_->setRecvSize(0);
                }
            } else {
                break;
            }
        }
    }

    const uint16_t frame_header_size = sizeof(FRAME_HEAD) + sizeof(ProtocolHeader);

    if (frame_->getRecvSize() >= 4 && frame_->getRecvSize() < frame_header_size) {
        uint32_t read_size = rx_buffer_->get(frame_->getData() + sizeof(FRAME_HEAD), sizeof(ProtocolHeader));
        frame_->addRecvSize(read_size);

        if (frame_->getRecvSize() != frame_header_size) {
            return;
        } else {
            ProtocolHeader header;
            std::memcpy(&header, frame_->getData() + sizeof(FRAME_HEAD), sizeof(ProtocolHeader));
            frame_->setHeader(header);
            frame_->setFrom(header.from);

#ifdef DEBUG
            std::printf("Header Info - From: %02x, To: %02x, Type: %02x, Index: %d, DSize: %d \n",
                       header.from, header.to, header.type, header.index, header.data_size);
            std::printf("%d self_node_id: %02x\n", __LINE__, self_node_id_);
#endif

            // Check if node ID is self
            if (header.to == self_node_id_) {
                switch (static_cast<ProtocolType>(header.type)) {
                case ProtocolType::HEART_PONG:
                    frame_->setDataSize(0);
                    break;
                case ProtocolType::SERVICE_RESPONSE_ERROR:
                    frame_->setDataSize(header.data_size);
                    break;
                case ProtocolType::SERVICE_REQUEST_READ:
                case ProtocolType::SERVICE_RESPONSE_READ:
                case ProtocolType::TRANSPORT_DATA:
                    frame_->setDataSize(header.data_size);
                    if (get_node_info_) {
                        frame_->setProtocolData(get_node_info_(header.index));
                        if (frame_->getProtocolData() == nullptr) {
#ifdef DEBUG
                            std::printf("No this index\n");
#endif
                            frame_->setDataSize(0);
                            frame_->setRecvSize(0);
                        }
                    }
                    break;
                default:
                    frame_->setDataSize(0);
                    frame_->setRecvSize(0);
                    break;
                }
            }
            // Check if message from other nodes
            else if (getOtherNodeData(header.from, header.index) != nullptr) {
                frame_->setProtocolData(getOtherNodeData(header.from, header.index));
                switch (static_cast<ProtocolType>(header.type)) {
                case ProtocolType::SERVICE_RESPONSE_ERROR:
                    frame_->setDataSize(header.data_size);
                    break;
                case ProtocolType::SERVICE_REQUEST_WRITE:
                    frame_->setDataSize(header.data_size);
                    break;
                case ProtocolType::SERVICE_RESPONSE_READ:
                case ProtocolType::TRANSPORT_DATA:
                    frame_->setDataSize(header.data_size);
                    break;
                case ProtocolType::HEART_PONG:
                    frame_->setDataSize(0);
                    break;
                default:
                    frame_->setDataSize(0);
                    frame_->setRecvSize(0);
                    break;
                }
            } else {
                frame_->setDataSize(0);
                frame_->setRecvSize(0);
            }
        }
    }

    if (frame_->getRecvSize() >= frame_header_size && frame_->getRecvSize() < (frame_header_size + frame_->getDataSize())) {
        uint32_t read_size = rx_buffer_->get(frame_->getData() + frame_->getRecvSize(), frame_->getDataSize());
        frame_->addRecvSize(read_size);
    }

    if (frame_->getRecvSize() >= frame_header_size + frame_->getDataSize()) {
        uint32_t read_size = rx_buffer_->get(frame_->getData() + frame_->getRecvSize(), 2);
        frame_->addRecvSize(read_size);

        if (frame_->getRecvSize() == (frame_header_size + frame_->getDataSize() + 2)) {
            uint16_t calculated_checksum = checksum16(frame_->getData(), frame_header_size + frame_->getDataSize());
            uint16_t received_checksum = (frame_->getData()[frame_header_size + frame_->getDataSize()] << 8) |
                                       frame_->getData()[frame_header_size + frame_->getDataSize() + 1];
            if (calculated_checksum == received_checksum) {
                processRequest();
            }
            frame_->setRecvSize(0);
        }
    }

    if (frame_->getRecvSize() > FPROTOCOL_FRAME_DATA_SIZE) {
        frame_->setRecvSize(0);
    }
}

void Handler::processRequest() {
    static uint8_t send_buff[FPROTOCOL_FRAME_DATA_SIZE];
    uint16_t ret = 0;
    std::memcpy(send_buff, FRAME_HEAD, 4);
    
    const auto& header = frame_->getHeader();
    const uint16_t frame_header_size = sizeof(FRAME_HEAD) + sizeof(ProtocolHeader);

    // Slave node
    if (header.to == self_node_id_) {
#ifdef DEBUG
        std::printf("Type: %02x\n", header.type);
        std::printf("frame->header.from: %02x frame.protocol_data: %p\n", header.from, frame_->getProtocolData());
#endif
        if (static_cast<ProtocolType>(header.type) == ProtocolType::HEART_PONG) {
            if (heart_ping_callback_) {
                heart_ping_callback_(header.from);
            }
            return;
        }
        
        // Need to write data
        if (static_cast<ProtocolType>(header.type) == ProtocolType::SERVICE_REQUEST_WRITE ||
            static_cast<ProtocolType>(header.type) == ProtocolType::TRANSPORT_DATA) {
            if (frame_->getProtocolData() && frame_->getProtocolData()->getStructDescriptor()) {
                unpackStruct(frame_->getData() + frame_header_size, frame_->getProtocolData()->getData(),
                           *frame_->getProtocolData()->getStructDescriptor());
            }
        }
        
        // Callback
        if (frame_->getProtocolData() && frame_->getProtocolData()->getCallback()) {
            ret = frame_->getProtocolData()->getCallback()(header.type, frame_->getFrom(), frame_->getErrorCode());
        }
        
        // Need to reply SERVICE_REQUEST_WRITE
        if (static_cast<ProtocolType>(header.type) == ProtocolType::SERVICE_REQUEST_WRITE) {
            if (ret == 0) {
                write(header.from, ProtocolType::SERVICE_RESPONSE_WRITE, header.index, nullptr, 0);
            } else {
                write(header.from, ProtocolType::SERVICE_RESPONSE_ERROR, header.index, &ret, 2);
            }
        }

        // Need to reply SERVICE_REQUEST_READ
        if (static_cast<ProtocolType>(header.type) == ProtocolType::SERVICE_REQUEST_READ) {
            if (frame_->getProtocolData()) {
                write(header.from, ProtocolType::SERVICE_RESPONSE_READ, header.index,
                     frame_->getProtocolData()->getData(), frame_->getProtocolData()->getDataSize(),
                     frame_->getProtocolData()->getStructDescriptor());
            }
        }
    }
    else { // Master node
        // Need to write data
        if (static_cast<ProtocolType>(header.type) == ProtocolType::SERVICE_RESPONSE_READ ||
            static_cast<ProtocolType>(header.type) == ProtocolType::TRANSPORT_DATA) {
            std::memcpy(frame_->getProtocolData()->getData(), frame_->getData() + frame_header_size, frame_->getDataSize());
        }
        
        // Callback
        if (frame_->getProtocolData() && frame_->getProtocolData()->getCallback()) {
            ret = frame_->getProtocolData()->getCallback()(header.type, frame_->getFrom(), frame_->getErrorCode());
        }
    }
}

size_t Handler::packStruct(uint8_t* buffer, const void* data, const StructDescriptor& desc) {
    size_t offset = 0;
    for (const auto& field : desc.fields) {
        const uint8_t* field_ptr = static_cast<const uint8_t*>(data) + field.offset;
        size_t len = 0;

        if (field.size_field >= 0) {
            // Variable length array
            const auto& size_field = desc.fields[field.size_field];
            const uint8_t* size_ptr = static_cast<const uint8_t*>(data) + size_field.offset;
            len = *size_ptr;
        } else if (field.array_len > 0) {
            // Fixed length array
            len = field.array_len;
        } else {
            // Single value field
            switch (field.type) {
            case FieldType::UINT8:
                len = 1;
                break;
            case FieldType::UINT16:
                len = 2;
                break;
            case FieldType::UINT32:
                len = 4;
                break;
            case FieldType::FLOAT:
                len = 4;
                break;
            default:
                break;
            }
        }

        std::memcpy(buffer + offset, field_ptr, len);
        offset += len;
    }
    return offset;
}

size_t Handler::unpackStruct(const uint8_t* buffer, void* data, const StructDescriptor& desc) {
#ifdef DEBUG
    std::printf("unpack_struct ");
#endif
    size_t offset = 0;
    for (const auto& field : desc.fields) {
        uint8_t* field_ptr = static_cast<uint8_t*>(data) + field.offset;
        size_t len = 0;

        if (field.size_field >= 0) {
            // Variable length array
            const auto& size_field = desc.fields[field.size_field];
            const uint8_t* size_ptr = static_cast<const uint8_t*>(data) + size_field.offset;
            len = *size_ptr;
        } else if (field.array_len > 0) {
            len = field.array_len;
        } else {
            switch (field.type) {
            case FieldType::UINT8:
                len = 1;
                break;
            case FieldType::UINT16:
                len = 2;
                break;
            case FieldType::UINT32:
                len = 4;
                break;
            case FieldType::FLOAT:
                len = 4;
                break;
            default:
                break;
            }
        }

        std::memcpy(field_ptr, buffer + offset, len);
        offset += len;
    }
    return offset;
}

uint16_t Handler::write(uint8_t to, ProtocolType type, uint16_t index,
                       const void* data, uint16_t size,
                       std::shared_ptr<StructDescriptor> desc) {
#ifdef DEBUG
    std::printf("write size:%d\n", size);
#endif
    static uint8_t send_buff[FPROTOCOL_FRAME_DATA_SIZE];
    std::memcpy(send_buff, FRAME_HEAD, 4);

    ProtocolHeader header;
    const uint16_t frame_header_size = sizeof(FRAME_HEAD) + sizeof(ProtocolHeader);
    header.from = self_node_id_;
    header.to = to;
    header.type = static_cast<uint8_t>(type);
    header.index = index;

    if (size && data) {
        if (desc) {
            size = packStruct(send_buff + frame_header_size, data, *desc);
        } else {
            std::memcpy(send_buff + frame_header_size, data, size);
        }
    }

    header.data_size = size;
    std::memcpy(send_buff + 4, &header, sizeof(ProtocolHeader));

#ifdef DEBUG
    std::printf("data_size: %02x\n", header.data_size);
#endif

    uint16_t checksum = checksum16(send_buff, frame_header_size + size);
    send_buff[frame_header_size + size] = checksum >> 8;
    send_buff[frame_header_size + size + 1] = checksum & 0xFF;

    return write_func_(to, send_buff, frame_header_size + size + 2);
}

int8_t Handler::heartPing() {
    return write(self_node_id_, ProtocolType::HEART_PING, 0, nullptr, 0) > 0;
}

void Handler::setHeartPingCallback(HeartPingCallback callback) {
    heart_ping_callback_ = callback;
}

uint16_t Handler::checksum16(const uint8_t* data, size_t length) {
    uint32_t sum = 0;
    for (size_t i = 0; i < length; i++) {
        sum += data[i];
        if (sum & 0xFFFF0000) {
            sum = (sum & 0xFFFF) + (sum >> 16);
        }
    }
    return ~(sum & 0xFFFF);
}

} // namespace FProtocol