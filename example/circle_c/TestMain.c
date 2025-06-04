#include "fprotocol.h"
#include "ExampleProto.h"
fprotocol_handler *handler;

int32_t mock_read(int16_t from, uint8_t *buf, int32_t size) {
    return 0;
}

int32_t mock_write(int16_t from, uint8_t *buf, int32_t size) {
    printf("Write data: %d-> ",size);
    for (int i = 0; i < size; i++) {
        printf("0x%02X,", buf[i]);
    }
    printf("\n");
    fprotocol_read_put(handler,buf, size);
    return size;
}

int16_t callback_read_485(uint16_t type, uint32_t from, uint16_t error_code)
{
    printf("Received 485 data: type=%u, from=%u, error_code=%u\n", type, from, error_code);
    printf("data=");
    for (int i = 0; i < read_485._data_size; i++) {
        printf("0x%02X ", read_485.data[i]);
    }
    printf("data1=");
    for (int i = 0; i < read_485._data1_size; i++) {
        printf("0x%02X ", read_485.data1[i]);
    }
    printf("\n");
    return 0;
}

int16_t callback_write_485(uint16_t type, uint32_t from, uint16_t error_code)
{
    return 0;
}

int main() {
    handler = fprotocol_init(mock_read, mock_write);
    fprotocol_set_host_node(handler, 1, example_index_info);
    // Initialize RS-485 data
    read_485._data_size = 18;
    for(uint8_t i = 0; i < read_485._data_size; i++) {
        read_485.data[i] = i;
    }
    read_485._data1_size = 5;
    for(uint8_t i = 0; i < read_485._data1_size; i++) {
        read_485.data1[i] = i;
    }
    write_read_485(handler, 1, 0);
    fprotocol_tick(handler);
    fprotocol_delete(handler);
    return 0;
}
