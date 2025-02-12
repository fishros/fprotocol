#ifndef FPROTOCOL_H
#define FPROTOCOL_H

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define RING_BUFFER_SIZE 1024
#define FPROTOCOL_FRAME_DATA_SIZE 1024
#define MAX_NODE 10

typedef int16_t (*fprotocol_callback)(uint16_t type, uint32_t from, uint16_t error_code);

typedef struct
{
    uint16_t index;
    uint16_t data_size;
    void *data;
    fprotocol_callback callback;
    // fprotocol_callback callback_response;
} fprotocol_data;

typedef fprotocol_data *(*fprotocol_get_index_info_t)(uint16_t index);

typedef enum
{
    SERVICE_REQUEST_WRITE = 0x01,
    SERVICE_REQUEST_READ, // Data 0Byte
    SERVICE_RESPONSE_WRITE,
    SERVICE_RESPONSE_READ,
    SERVICE_RESPONSE_ERROR, // Data 2byte -> Error code
    TRANSPORT_DATA,
    HEART,
    MAX,
} fprotocol_type;

typedef struct
{
    uint16_t node;
    uint8_t type;
    uint16_t index;
    uint16_t data_size; // Data size 2Byte
} __attribute__((packed)) fprotocol_header;

typedef struct
{
    fprotocol_header header;
    uint16_t data_size;
    uint16_t recv_size;
    int16_t from;
    uint16_t error_code;
    fprotocol_data *fdata; // 对应的fprotocol结构体，包含索引，回调函数等
    uint8_t *data;
} fprotocol_frame;

typedef struct
{
    uint8_t *buf;
    uint32_t bufSize;
    uint32_t pIn;
    uint32_t pOut;
} fring_buffer;

typedef struct
{
    uint16_t node;                                       // 当前节点号/设备号
    fprotocol_get_index_info_t fprotocol_get_node_info; // 从站节点的获取索引信息函数
    int32_t (*read)(int16_t, uint8_t *, int32_t);
    int32_t (*write)(int16_t, uint8_t *, int32_t);
    fprotocol_frame *frame;               // 接收frame相关准备
    fring_buffer *rxbuff;                 // 接收缓冲区
    uint8_t snode_count;                  // 已有节点数量
    uint16_t snode_index_table[MAX_NODE]; // 节点索引表
    fprotocol_get_index_info_t fprotocol_get_index_info_table[MAX_NODE];
} fprotocol_handler;

void fring_init(fring_buffer *buffer, uint32_t size);
int32_t fring_put(fring_buffer *p, uint8_t *buf, uint32_t size);
int32_t fring_get(fring_buffer *p, uint8_t *buf, uint32_t size);
uint32_t fring_size(fring_buffer *p);
void fring_clear(fring_buffer *p);

fprotocol_handler *fprotocol_init(int32_t (*read)(int16_t, uint8_t *, int32_t), int32_t (*write)(int16_t, uint8_t *, int32_t));
void fprotocol_set_host_node(fprotocol_handler *handler, uint8_t node, fprotocol_get_index_info_t get_index_info);
void fprotocol_add_slave_node(fprotocol_handler *handler, uint8_t node, fprotocol_get_index_info_t get_index_info);
fprotocol_data *fprotocol_get_slave_node_data(fprotocol_handler *handler, uint16_t node, uint16_t index);

void fprotocol_tick(fprotocol_handler *handler);
void fprotocol_read_put(fprotocol_handler *handler,uint8_t *buf, uint32_t size);
void fprotocol_req_deal(fprotocol_handler *handler);
void fprotocol_delete(fprotocol_handler *handler);
void fprotocol_write(fprotocol_handler *handler, uint16_t node, uint8_t type, uint16_t index, void *data, uint16_t size);
// void fprotocol_write_index(fprotocol_handler *handler, uint16_t index, uint8_t *data, uint16_t size);
uint16_t checksum16(const uint8_t *data, size_t length);
// extern fprotocol_get_index_info_t* fprotocol_get_index_info;

#endif // FPROTOCOL_H
