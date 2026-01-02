#include "RobotProto.hpp"
#include <cstring>  // for memset


// Struct descriptor for float3d_t
static std::vector<FProtocol::FieldDescriptor> float3d_t_fields = {
        FProtocol::FieldDescriptor(offsetof(float3d_t, x), FProtocol::FieldType::FLOAT, -1, 0),
        FProtocol::FieldDescriptor(offsetof(float3d_t, y), FProtocol::FieldType::FLOAT, -1, 0),
        FProtocol::FieldDescriptor(offsetof(float3d_t, z), FProtocol::FieldType::FLOAT, -1, 0)
};

static std::shared_ptr<FProtocol::StructDescriptor> float3d_t_desc = 
    std::make_shared<FProtocol::StructDescriptor>(sizeof(float3d_t), float3d_t_fields);

// Struct descriptor for data_t
static std::vector<FProtocol::FieldDescriptor> data_t_fields = {
        FProtocol::FieldDescriptor(offsetof(data_t, temp), FProtocol::FieldType::FLOAT, -1, 0),
        FProtocol::FieldDescriptor(offsetof(data_t, hum), FProtocol::FieldType::FLOAT, -1, 0),
        FProtocol::FieldDescriptor(offsetof(data_t, ladc), FProtocol::FieldType::UINT8, -1, 0),
        FProtocol::FieldDescriptor(offsetof(data_t, badc), FProtocol::FieldType::UINT8, -1, 0)
};

static std::shared_ptr<FProtocol::StructDescriptor> data_t_desc = 
    std::make_shared<FProtocol::StructDescriptor>(sizeof(data_t), data_t_fields);

// Struct descriptor for imu_t
static std::vector<FProtocol::FieldDescriptor> imu_t_fields = {
        FProtocol::FieldDescriptor(offsetof(imu_t, x), FProtocol::FieldType::FLOAT, -1, 0),
        FProtocol::FieldDescriptor(offsetof(imu_t, y), FProtocol::FieldType::FLOAT, -1, 0),
        FProtocol::FieldDescriptor(offsetof(imu_t, z), FProtocol::FieldType::FLOAT, -1, 0),
        FProtocol::FieldDescriptor(offsetof(imu_t, acc_x), FProtocol::FieldType::FLOAT, -1, 0),
        FProtocol::FieldDescriptor(offsetof(imu_t, acc_y), FProtocol::FieldType::FLOAT, -1, 0),
        FProtocol::FieldDescriptor(offsetof(imu_t, acc_z), FProtocol::FieldType::FLOAT, -1, 0),
        FProtocol::FieldDescriptor(offsetof(imu_t, gyro_x), FProtocol::FieldType::FLOAT, -1, 0),
        FProtocol::FieldDescriptor(offsetof(imu_t, gyro_y), FProtocol::FieldType::FLOAT, -1, 0),
        FProtocol::FieldDescriptor(offsetof(imu_t, gyro_z), FProtocol::FieldType::FLOAT, -1, 0)
};

static std::shared_ptr<FProtocol::StructDescriptor> imu_t_desc = 
    std::make_shared<FProtocol::StructDescriptor>(sizeof(imu_t), imu_t_fields);

static std::vector<FProtocol::FieldDescriptor> uint8_t_fields = {
    FProtocol::FieldDescriptor(0, FProtocol::FieldType::UINT8, -1, 0)
};

static std::shared_ptr<FProtocol::StructDescriptor> uint8_t_desc = 
    std::make_shared<FProtocol::StructDescriptor>(sizeof(uint8_t), uint8_t_fields);

RobotProtocol::RobotProtocol() {
    // Initialize member variables
    ctrl_led_ = 0;
    ctrl_beep_ = 0;
    memset(&ctrl_vel_, 0, sizeof(ctrl_vel_));
    memset(&sensor_odom_, 0, sizeof(sensor_odom_));
    memset(&sensor_board_, 0, sizeof(sensor_board_));
    memset(&sensor_imu_, 0, sizeof(sensor_imu_));
    config_odom_hz_ = 0;
    config_imu_hz_ = 0;
    config_sensor_hz_ = 0;
    ctrl_led_callback_ = nullptr;
    ctrl_beep_callback_ = nullptr;
    ctrl_vel_callback_ = nullptr;
    sensor_board_callback_ = nullptr;
    config_odom_hz_callback_ = nullptr;
    config_imu_hz_callback_ = nullptr;
    config_sensor_hz_callback_ = nullptr;
}

FProtocol::ProtocolData* RobotProtocol::getIndexInfo(uint16_t index) {
    static std::vector<std::unique_ptr<FProtocol::ProtocolData>> protocol_data_instances;
    if (protocol_data_instances.empty()) {
        protocol_data_instances.push_back(std::make_unique<FProtocol::ProtocolData>(0x0001, sizeof(ctrl_led_), &ctrl_led_, [this](uint16_t type, uint32_t from, uint16_t error_code) -> int16_t { return ctrl_led_callback_ ? ctrl_led_callback_(type, from, error_code) : 0; }, uint8_t_desc));
        protocol_data_instances.push_back(std::make_unique<FProtocol::ProtocolData>(0x0002, sizeof(ctrl_beep_), &ctrl_beep_, [this](uint16_t type, uint32_t from, uint16_t error_code) -> int16_t { return ctrl_beep_callback_ ? ctrl_beep_callback_(type, from, error_code) : 0; }, uint8_t_desc));
        protocol_data_instances.push_back(std::make_unique<FProtocol::ProtocolData>(0x0003, sizeof(ctrl_vel_), &ctrl_vel_, [this](uint16_t type, uint32_t from, uint16_t error_code) -> int16_t { return ctrl_vel_callback_ ? ctrl_vel_callback_(type, from, error_code) : 0; }, float3d_t_desc));
        protocol_data_instances.push_back(std::make_unique<FProtocol::ProtocolData>(0x0004, sizeof(sensor_odom_), &sensor_odom_, nullptr, float3d_t_desc));
        protocol_data_instances.push_back(std::make_unique<FProtocol::ProtocolData>(0x0005, sizeof(sensor_board_), &sensor_board_, [this](uint16_t type, uint32_t from, uint16_t error_code) -> int16_t { return sensor_board_callback_ ? sensor_board_callback_(type, from, error_code) : 0; }, data_t_desc));
        protocol_data_instances.push_back(std::make_unique<FProtocol::ProtocolData>(0x0006, sizeof(sensor_imu_), &sensor_imu_, nullptr, imu_t_desc));
        protocol_data_instances.push_back(std::make_unique<FProtocol::ProtocolData>(0x0101, sizeof(config_odom_hz_), &config_odom_hz_, [this](uint16_t type, uint32_t from, uint16_t error_code) -> int16_t { return config_odom_hz_callback_ ? config_odom_hz_callback_(type, from, error_code) : 0; }, uint8_t_desc));
        protocol_data_instances.push_back(std::make_unique<FProtocol::ProtocolData>(0x0102, sizeof(config_imu_hz_), &config_imu_hz_, [this](uint16_t type, uint32_t from, uint16_t error_code) -> int16_t { return config_imu_hz_callback_ ? config_imu_hz_callback_(type, from, error_code) : 0; }, uint8_t_desc));
        protocol_data_instances.push_back(std::make_unique<FProtocol::ProtocolData>(0x0103, sizeof(config_sensor_hz_), &config_sensor_hz_, [this](uint16_t type, uint32_t from, uint16_t error_code) -> int16_t { return config_sensor_hz_callback_ ? config_sensor_hz_callback_(type, from, error_code) : 0; }, uint8_t_desc));
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

void RobotProtocol::write_ctrl_led(FProtocol::Handler* handler, uint16_t node, bool response) {
    handler->write(node, response ? FProtocol::ProtocolType::SERVICE_REQUEST_WRITE : FProtocol::ProtocolType::TRANSPORT_DATA, 
                   0x0001, &ctrl_led_, sizeof(ctrl_led_), uint8_t_desc);
}

void RobotProtocol::read_ctrl_led(FProtocol::Handler* handler, uint16_t node) {
    handler->write(node, FProtocol::ProtocolType::SERVICE_REQUEST_READ, 0x0001, nullptr, 0, uint8_t_desc);
}

void RobotProtocol::write_ctrl_beep(FProtocol::Handler* handler, uint16_t node, bool response) {
    handler->write(node, response ? FProtocol::ProtocolType::SERVICE_REQUEST_WRITE : FProtocol::ProtocolType::TRANSPORT_DATA, 
                   0x0002, &ctrl_beep_, sizeof(ctrl_beep_), uint8_t_desc);
}

void RobotProtocol::read_ctrl_beep(FProtocol::Handler* handler, uint16_t node) {
    handler->write(node, FProtocol::ProtocolType::SERVICE_REQUEST_READ, 0x0002, nullptr, 0, uint8_t_desc);
}

void RobotProtocol::write_ctrl_vel(FProtocol::Handler* handler, uint16_t node, bool response) {
    handler->write(node, response ? FProtocol::ProtocolType::SERVICE_REQUEST_WRITE : FProtocol::ProtocolType::TRANSPORT_DATA, 
                   0x0003, &ctrl_vel_, sizeof(ctrl_vel_), float3d_t_desc);
}

void RobotProtocol::read_ctrl_vel(FProtocol::Handler* handler, uint16_t node) {
    handler->write(node, FProtocol::ProtocolType::SERVICE_REQUEST_READ, 0x0003, nullptr, 0, float3d_t_desc);
}

void RobotProtocol::write_sensor_odom(FProtocol::Handler* handler, uint16_t node, bool response) {
    handler->write(node, response ? FProtocol::ProtocolType::SERVICE_REQUEST_WRITE : FProtocol::ProtocolType::TRANSPORT_DATA, 
                   0x0004, &sensor_odom_, sizeof(sensor_odom_), float3d_t_desc);
}

void RobotProtocol::read_sensor_odom(FProtocol::Handler* handler, uint16_t node) {
    handler->write(node, FProtocol::ProtocolType::SERVICE_REQUEST_READ, 0x0004, nullptr, 0, float3d_t_desc);
}

void RobotProtocol::write_sensor_board(FProtocol::Handler* handler, uint16_t node, bool response) {
    handler->write(node, response ? FProtocol::ProtocolType::SERVICE_REQUEST_WRITE : FProtocol::ProtocolType::TRANSPORT_DATA, 
                   0x0005, &sensor_board_, sizeof(sensor_board_), data_t_desc);
}

void RobotProtocol::read_sensor_board(FProtocol::Handler* handler, uint16_t node) {
    handler->write(node, FProtocol::ProtocolType::SERVICE_REQUEST_READ, 0x0005, nullptr, 0, data_t_desc);
}

void RobotProtocol::write_sensor_imu(FProtocol::Handler* handler, uint16_t node, bool response) {
    handler->write(node, response ? FProtocol::ProtocolType::SERVICE_REQUEST_WRITE : FProtocol::ProtocolType::TRANSPORT_DATA, 
                   0x0006, &sensor_imu_, sizeof(sensor_imu_), imu_t_desc);
}

void RobotProtocol::read_sensor_imu(FProtocol::Handler* handler, uint16_t node) {
    handler->write(node, FProtocol::ProtocolType::SERVICE_REQUEST_READ, 0x0006, nullptr, 0, imu_t_desc);
}

void RobotProtocol::write_config_odom_hz(FProtocol::Handler* handler, uint16_t node, bool response) {
    handler->write(node, response ? FProtocol::ProtocolType::SERVICE_REQUEST_WRITE : FProtocol::ProtocolType::TRANSPORT_DATA, 
                   0x0101, &config_odom_hz_, sizeof(config_odom_hz_), uint8_t_desc);
}

void RobotProtocol::read_config_odom_hz(FProtocol::Handler* handler, uint16_t node) {
    handler->write(node, FProtocol::ProtocolType::SERVICE_REQUEST_READ, 0x0101, nullptr, 0, uint8_t_desc);
}

void RobotProtocol::write_config_imu_hz(FProtocol::Handler* handler, uint16_t node, bool response) {
    handler->write(node, response ? FProtocol::ProtocolType::SERVICE_REQUEST_WRITE : FProtocol::ProtocolType::TRANSPORT_DATA, 
                   0x0102, &config_imu_hz_, sizeof(config_imu_hz_), uint8_t_desc);
}

void RobotProtocol::read_config_imu_hz(FProtocol::Handler* handler, uint16_t node) {
    handler->write(node, FProtocol::ProtocolType::SERVICE_REQUEST_READ, 0x0102, nullptr, 0, uint8_t_desc);
}

void RobotProtocol::write_config_sensor_hz(FProtocol::Handler* handler, uint16_t node, bool response) {
    handler->write(node, response ? FProtocol::ProtocolType::SERVICE_REQUEST_WRITE : FProtocol::ProtocolType::TRANSPORT_DATA, 
                   0x0103, &config_sensor_hz_, sizeof(config_sensor_hz_), uint8_t_desc);
}

void RobotProtocol::read_config_sensor_hz(FProtocol::Handler* handler, uint16_t node) {
    handler->write(node, FProtocol::ProtocolType::SERVICE_REQUEST_READ, 0x0103, nullptr, 0, uint8_t_desc);
}

