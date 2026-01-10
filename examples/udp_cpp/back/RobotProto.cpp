#include "RobotProto.hpp"
#include <cstring>  // for memset


// Struct descriptor for data_t
static std::vector<FProtocol::FieldDescriptor> data_t_fields = {
        FProtocol::FieldDescriptor(offsetof(data_t, status_charge), FProtocol::FieldType::UINT8, -1, 0),
        FProtocol::FieldDescriptor(offsetof(data_t, volatge), FProtocol::FieldType::UINT8, -1, 0)
};

static std::shared_ptr<FProtocol::StructDescriptor> data_t_desc = 
    std::make_shared<FProtocol::StructDescriptor>(sizeof(data_t), data_t_fields);

static std::vector<FProtocol::FieldDescriptor> uint8_t_fields = {
    FProtocol::FieldDescriptor(0, FProtocol::FieldType::UINT8, -1, 0)
};

static std::shared_ptr<FProtocol::StructDescriptor> uint8_t_desc = 
    std::make_shared<FProtocol::StructDescriptor>(sizeof(uint8_t), uint8_t_fields);

RobotProtocol::RobotProtocol() {
    // Initialize member variables
    led_ = 0;
    memset(&data_, 0, sizeof(data_));
    data_callback_ = nullptr;
}

FProtocol::ProtocolData* RobotProtocol::getIndexInfo(uint16_t index) {
    static std::vector<std::unique_ptr<FProtocol::ProtocolData>> protocol_data_instances;
    if (protocol_data_instances.empty()) {
        protocol_data_instances.push_back(std::make_unique<FProtocol::ProtocolData>(0x0001, sizeof(led_), &led_, nullptr, uint8_t_desc));
        protocol_data_instances.push_back(std::make_unique<FProtocol::ProtocolData>(0x02, sizeof(data_), &data_, [this](uint16_t type, uint8_t from, uint16_t error_code) -> int16_t { return data_callback_ ? data_callback_(type, from, error_code) : 0; }, data_t_desc));
    }
    
    for (auto& data : protocol_data_instances) {
        if (data->getIndex() == index) {
            return data.get();
        }
    }
    return nullptr;
}

FProtocol::GetIndexInfoFunction RobotProtocol::getIndexInfoFunction() {
    return [this](uint16_t index) -> FProtocol::ProtocolData* { return this->getIndexInfo(index); };
}

void RobotProtocol::write_led(FProtocol::Handler* handler, uint8_t node, bool response) {
    handler->write(node, response ? FProtocol::ProtocolType::SERVICE_REQUEST_WRITE : FProtocol::ProtocolType::TRANSPORT_DATA,
                   0x0001, &led_, sizeof(led_), uint8_t_desc);
}

void RobotProtocol::read_led(FProtocol::Handler* handler, uint8_t node) {
    handler->write(node, FProtocol::ProtocolType::SERVICE_REQUEST_READ, 0x0001, nullptr, 0, uint8_t_desc);
}

void RobotProtocol::write_data(FProtocol::Handler* handler, uint8_t node, bool response) {
    handler->write(node, response ? FProtocol::ProtocolType::SERVICE_REQUEST_WRITE : FProtocol::ProtocolType::TRANSPORT_DATA,
                   0x02, &data_, sizeof(data_), data_t_desc);
}

void RobotProtocol::read_data(FProtocol::Handler* handler, uint8_t node) {
    handler->write(node, FProtocol::ProtocolType::SERVICE_REQUEST_READ, 0x02, nullptr, 0, data_t_desc);
}

