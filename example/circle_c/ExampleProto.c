#include "ExampleProto.h"

rawuint8_t write_485;
rawuint8_t read_485;
uint8_t test;

static const FieldDescriptor rawuint8_t_fields[] = {
    {offsetof(rawuint8_t, _data_size), TYPE_UINT16, -1, 0},
    {offsetof(rawuint8_t, data), TYPE_UINT8, 0, 1024},
    {offsetof(rawuint8_t, _data1_size), TYPE_UINT16, -1, 0},
    {offsetof(rawuint8_t, data1), TYPE_UINT8, 2, 1024},
};

static const StructDescriptor rawuint8_t_desc = {
    .size = sizeof(rawuint8_t),
    .field_count = 4,
    .fields = rawuint8_t_fields,
};

fprotocol_data data_table[] = {
    {0x0001, sizeof(write_485), &write_485, callback_write_485,&rawuint8_t_desc},
    {0x0002, sizeof(read_485), &read_485, callback_read_485,&rawuint8_t_desc},
    {0x0003, sizeof(test), &test, NULL,NULL},
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
    case 0x0003:
        return &data_table[2];
        break;
    default:
        break;
    }
    return NULL;
}
void write_write_485(fprotocol_handler *handler,uint16_t node,uint8_t response)
{
    fprotocol_write(handler, node, response ? SERVICE_REQUEST_WRITE : TRANSPORT_DATA, 0x0001, &write_485, sizeof(write_485),&rawuint8_t_desc);
}
void read_write_485(fprotocol_handler *handler,uint16_t node)
{
    fprotocol_write(handler, node, SERVICE_REQUEST_READ, 0x0001, &write_485, sizeof(write_485),&rawuint8_t_desc);
}
void write_read_485(fprotocol_handler *handler,uint16_t node,uint8_t response)
{
    fprotocol_write(handler, node, response ? SERVICE_REQUEST_WRITE : TRANSPORT_DATA, 0x0002, &read_485, sizeof(read_485),&rawuint8_t_desc);
}
void read_read_485(fprotocol_handler *handler,uint16_t node)
{
    fprotocol_write(handler, node, SERVICE_REQUEST_READ, 0x0002, &read_485, sizeof(read_485),&rawuint8_t_desc);
}
void write_test(fprotocol_handler *handler,uint16_t node,uint8_t response)
{
    fprotocol_write(handler, node, response ? SERVICE_REQUEST_WRITE : TRANSPORT_DATA, 0x0003, &test, sizeof(test),NULL);
}
void read_test(fprotocol_handler *handler,uint16_t node)
{
    fprotocol_write(handler, node, SERVICE_REQUEST_READ, 0x0003, &test, sizeof(test),NULL);
}
fprotocol_get_index_info_t example_index_info = example_fprotocol_get_index_info;