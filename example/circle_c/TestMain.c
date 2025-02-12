#include "fprotocol.h"
#include "ExampleProto.h"
fprotocol_handler *handler;

int32_t mock_read(int16_t from, uint8_t *buf, int32_t size) {
    // printf("Read data: ");
    // for (int i = 0; i < size; i++) {
    //     printf("%02X ", buf[i]);
    // }
    // printf("\n");
    return 0;
}

int32_t mock_write(int16_t from, uint8_t *buf, int32_t size) {
    printf("Write data: %d-> ",size);
    // Mock write function
    for (int i = 0; i < size; i++) {
        printf("0x%02X,", buf[i]);
    }
    printf("\n");
    fprotocol_read_put(handler,buf, size);
    return size;
}

int16_t callback_can_recv_frame(uint16_t type, uint32_t from, uint16_t error_code)
{
    printf("Received CAN frame: type=%u, from=%u, error_code=%u\n", type, from, error_code);
    printf("arbitration_id=%u, dlc=%u, rtr=%u, data=", can_recv_frame.arbitration_id, can_recv_frame.dlc, can_recv_frame.rtr);
    for (int i = 0; i < can_recv_frame.dlc; i++) {
        printf("0x%02X ", can_recv_frame.data[i]);
    }
    printf("\n");
    return 0;
}

int16_t callback_can_send_frame(uint16_t type, uint32_t from, uint16_t error_code)
{

    return 0;
}



int main() {
    handler = fprotocol_init(mock_read, mock_write);
    fprotocol_set_host_node(handler, 1, example_index_info);

    can_recv_frame.arbitration_id = 0x0002;
    can_recv_frame.dlc = 8;
    for(uint8_t i = 0; i < 8; i++) {
        can_recv_frame.data[i] = i;
    }
    write_can_recv_frame(handler, 1, 0);
    

    // while (1)
    // {
    fprotocol_tick(handler);
    // }
    // fprotocol_tick(handler);
    // fprotocol_req_deal(handler);

    fprotocol_delete(handler);
    return 0;
}