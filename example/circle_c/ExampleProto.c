#include "ExampleProto.h"

canframe_t can_recv_frame;
canframe_t can_send_frame;

fprotocol_data data_table[] = {
    {0x0001, sizeof(can_recv_frame), &can_recv_frame, callback_can_recv_frame},
    {0x0002, sizeof(can_send_frame), &can_send_frame, callback_can_send_frame},
};

fprotocol_data *example_fprotocol_get_index_info(uint16_t index)
{
    switch (index)
    {
    case 0x0001:
        return &data_table[0];
        break;
    case 0x0002:
        return &data_table[1];
        break;
    default:
        break;
    }
    return NULL;
}
void write_can_recv_frame(fprotocol_handler *handler,uint16_t node,uint8_t response)
{
    fprotocol_write(handler, node, response ? SERVICE_REQUEST_WRITE : TRANSPORT_DATA, 0x0001, &can_recv_frame, sizeof(can_recv_frame));
}
void read_can_recv_frame(fprotocol_handler *handler,uint16_t node)
{
    fprotocol_write(handler, node, SERVICE_REQUEST_READ, 0x0001, &can_recv_frame, sizeof(can_recv_frame));
}
void write_can_send_frame(fprotocol_handler *handler,uint16_t node,uint8_t response)
{
    fprotocol_write(handler, node, response ? SERVICE_REQUEST_WRITE : TRANSPORT_DATA, 0x0002, &can_send_frame, sizeof(can_send_frame));
}
void read_can_send_frame(fprotocol_handler *handler,uint16_t node)
{
    fprotocol_write(handler, node, SERVICE_REQUEST_READ, 0x0002, &can_send_frame, sizeof(can_send_frame));
}
fprotocol_get_index_info_t example_index_info = example_fprotocol_get_index_info;