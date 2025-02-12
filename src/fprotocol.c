#include "fprotocol.h"

static uint8_t FRAME_HEAD[4] = {0x55, 0xAA, 0x55, 0xAA};
// #define DEBUG

void fprotocol_add_slave_node(fprotocol_handler *handler, uint8_t node, fprotocol_get_index_info_t get_index_info)
{
    if (handler->snode_count >= MAX_NODE)
    {
        return;
    }
    handler->snode_index_table[handler->snode_count] = node;
    handler->fprotocol_get_index_info_table[handler->snode_count] = get_index_info;
    handler->snode_count++;
}

fprotocol_data *fprotocol_get_slave_node_data(fprotocol_handler *handler, uint16_t node, uint16_t index)
{
    printf("%d\n",__LINE__);
    if(handler->snode_count == 0)
    {
        return NULL;
    }
    for (int i = 0; i < handler->snode_count; i++)
    {
        printf("%d\n",__LINE__);

        if (handler->snode_index_table[i] == node)
        {
            printf("%d\n",__LINE__);
            // printf("%d:%p,%p\n",__LINE__,*handler,handler->fprotocol_get_index_info_table[i]);
            fprotocol_data * data = handler->fprotocol_get_index_info_table[i](index);
            printf("%d\n",__LINE__);
            return data;
        }
    }
    return NULL;
}

void fprotocol_read_put(fprotocol_handler *handler,uint8_t *buf, uint32_t size)
{
    fring_put(handler->rxbuff, buf, size);
    #ifdef DEBUG    
    printf("put data to ring_buffer_size: %d %s\n", size,buf);
    #endif
}

void fprotocol_tick(fprotocol_handler *handler)
{
    static uint8_t data[256];
    static uint16_t from = 0;
    uint8_t rdata;
    fprotocol_frame *frame = handler->frame;
    if (handler->read)
    {
        int16_t rsize = handler->read(from, data, 256);
        if (rsize)
        {
            fring_put(handler->rxbuff, data, rsize);
#ifdef DEBUG
            printf("put data to ring_buffer_size: %d\n", rsize);
#endif
        }
    }
#ifdef DEBUG
    printf("%d:%p\n",__LINE__,*handler);
    printf("fring_size: %d frame->recv_size:%u\n", fring_size(handler->rxbuff),frame->recv_size);
#endif
    if (frame->recv_size < 4)
    {
#ifdef DEBUG
        printf("frame->recv_size: %d\n", frame->recv_size);
#endif
        while (frame->recv_size < 4)
        {
            if (fring_get(handler->rxbuff, &rdata, 1))
            {
                #ifdef DEBUG
                printf("rdata: %02x\n", rdata);
#endif
                if (rdata == FRAME_HEAD[frame->recv_size])
                {
                    frame->data[frame->recv_size] = rdata;
                    frame->recv_size++;
                }
                else if (rdata == FRAME_HEAD[0])
                {
                    frame->data[0] = rdata;
                    frame->recv_size = 1;
                }
                else
                {
                    frame->recv_size = 0;
                }
            }
            else
            {
                printf("fring_size(handler->rxbuff)=%d\n",fring_size(handler->rxbuff));
                break;
            }
        }
    }
    if (frame->recv_size >= 4 && frame->recv_size < 11)
    {
        frame->recv_size += fring_get(handler->rxbuff, frame->data + sizeof(FRAME_HEAD), sizeof(fprotocol_header));
        if (frame->recv_size != 11)
        {
            return;
        }
        else
        {
            frame->from = from;
            memcpy(&frame->header, frame->data + sizeof(FRAME_HEAD), sizeof(fprotocol_header));
#ifdef DEBUG
            printf("Header Info - Node: %02x, Type: %02x, Index: %d, DSize: %d \n",
                   frame->header.node, frame->header.type, frame->header.index, frame->header.data_size);
            printf("%d\n",__LINE__);
#endif

            // 判断nodeid是否是当前节点，是表示是从站
            if (frame->header.node == handler->node)
            {
                // printf("%d\n",__LINE__);

                frame->fdata = (handler->fprotocol_get_node_info)(frame->header.index);
                if (frame->fdata == NULL) // 无此索引
                {
#ifdef DEBUG
                    printf("No this index\n");
#endif

                    frame->data_size = 0;
                    frame->recv_size = 0;
                }
                else
                {
                    #ifdef DEBUG
                    printf("Get Data Size: %d\n",frame->header.data_size );
#endif
                    frame->data_size = frame->header.data_size;
                    // switch (frame->header.type)
                    // {
                    // case SERVICE_RESPONSE_ERROR:
                    //     frame->data_size = frame->header.data_size; // Error code
                    //     break;
                    // case SERVICE_RESPONSE_READ:
                    //     frame->data_size = frame->header.data_size;
                    //     break;
                    // default:
                    //     frame->data_size = frame->fdata->data_size;
                    //     break;
                    // }
                }
            }
            // 判断是否是来自从站的消息/从站响应错误/从站请求写入/从站响应读取/从站传输数据
            else if (fprotocol_get_slave_node_data(handler, frame->header.node, frame->header.index) != NULL)
            {
                printf("%d\n",__LINE__);
                frame->fdata = fprotocol_get_slave_node_data(handler, frame->header.node, frame->header.index);
                switch (frame->header.type)
                {
                case SERVICE_RESPONSE_ERROR:
                    frame->data_size = frame->header.data_size; // Error code
                    break;
                case SERVICE_REQUEST_WRITE:
                    frame->data_size = frame->header.data_size;
                    break;
                // case SERVICE_REQUEST_WRITE:
                // case SERVICE_REQUEST_READ:
                case SERVICE_RESPONSE_READ:
                case TRANSPORT_DATA:
                    frame->data_size = frame->header.data_size;
                    break;
                default:
                    // reset
                    frame->data_size = 0;
                    frame->recv_size = 0;
                    break;
                }
            }
            else
            {
                frame->data_size = 0;
                frame->recv_size = 0;
            }
        }
    }

    if (frame->recv_size >= 11 && frame->recv_size < (11 + frame->data_size))
    {
        frame->recv_size += fring_get(handler->rxbuff, frame->data + frame->recv_size, frame->data_size);
    }

    if (frame->recv_size >= 11 + frame->data_size)
    {
        frame->recv_size += fring_get(handler->rxbuff, frame->data + frame->recv_size, 2);
        if (frame->recv_size == (13 + frame->data_size))
        {
            uint16_t calculated_checksum = checksum16(frame->data, 11 + frame->data_size);
            uint16_t received_checksum = (frame->data[11 + frame->data_size] << 8) | frame->data[12 + frame->data_size];
            if (calculated_checksum == received_checksum)
            {
                fprotocol_req_deal(handler);
            }
            frame->recv_size = 0;
        }
    }

    if(frame->recv_size >(FPROTOCOL_FRAME_DATA_SIZE))
    {
        frame->recv_size = 0;
    }
}

void fprotocol_delete(fprotocol_handler *handler)
{
    if (handler != NULL)
    {
        if (handler->frame != NULL)
        {
            free(handler->frame);
        }
        if (handler->rxbuff->buf != NULL)
        {
            free(handler->rxbuff->buf);
            free(handler->rxbuff);
        }
        free(handler);
    }
}

void fprotocol_req_deal(fprotocol_handler *handler)
{
    static uint8_t send_buff[FPROTOCOL_FRAME_DATA_SIZE];
    fprotocol_frame *frame = handler->frame;
    uint16_t ret = 0;
    memcpy(send_buff, FRAME_HEAD, 4);
    // 从站
    if (frame->header.node == handler->node)
    {
#ifdef DEBUG
        printf("Type: %02x\n", frame->header.type);
#endif
        // 需要写入数据的情况
        if (frame->header.type == SERVICE_REQUEST_WRITE || frame->header.type == TRANSPORT_DATA)
        {
            memcpy(frame->fdata->data, frame->data + 11, frame->data_size); // 拷贝数据到指定节点数据体 TODO: 确认frame->fdata->data 大小足够放下
        }
        // 回调
        if (frame->fdata->callback != NULL)
        {
            ret = frame->fdata->callback(frame->header.type, frame->from, frame->error_code);
        }
        // 需要回复1 SERVICE_REQUEST_WRITE
        if (frame->header.type == SERVICE_REQUEST_WRITE)
        {
            if (ret == 0)
            {
                // frame->header.type = SERVICE_RESPONSE_WRITE;
                // memcpy(send_buff + 4, &frame->header, sizeof(fprotocol_header));
                // uint16_t checksum = checksum16(send_buff, 9);
                // send_buff[9] = checksum >> 8;
                // send_buff[10] = checksum & 0xFF;
                // handler->write(frame->from, send_buff, 11);
                fprotocol_write(handler, frame->header.node, SERVICE_RESPONSE_WRITE, frame->header.index, NULL, 0);
            }
            else
            {
                // frame->header.type = SERVICE_RESPONSE_ERROR;
                // memcpy(send_buff + 4, &frame->header, sizeof(fprotocol_header));
                // memcpy(send_buff + 9, &ret, 2);
                // uint16_t checksum = checksum16(send_buff, 11);
                // send_buff[11] = checksum >> 8;
                // send_buff[12] = checksum & 0xFF;
                // handler->write(frame->from, send_buff, 13);
                fprotocol_write(handler, frame->header.node, SERVICE_RESPONSE_ERROR, frame->header.index, &ret, 2);
            }
        }
        // 需要回复2 SERVICE_REQUEST_WRITE
        if (frame->header.type == SERVICE_REQUEST_READ)
        {
            // frame->header.type = SERVICE_RESPONSE_READ;
            // memcpy(send_buff + 4, &frame->header, sizeof(fprotocol_header));
            // memcpy(send_buff + 9, frame->fdata->data, frame->fdata->data_size);
            // uint16_t checksum = checksum16(send_buff, 9 + frame->fdata->data_size);
            // send_buff[9 + frame->fdata->data_size + 1] = checksum >> 8;
            // send_buff[9 + frame->fdata->data_size + 2] = checksum & 0xFF;
            // handler->write(frame->from, send_buff, 9 + frame->fdata->data_size + 2);
            fprotocol_write(handler, frame->header.node, SERVICE_RESPONSE_READ, frame->header.index, frame->fdata->data, frame->fdata->data_size);
        }
    }
    else // 能到这里一定是主站了
    {
        // 需要写入数据的情况
        if (frame->header.type == SERVICE_RESPONSE_READ || frame->header.type == TRANSPORT_DATA)
        {
            memcpy(frame->fdata->data, frame->data + 9, frame->data_size); // 拷贝数据到指定节点数据体
        }
        // 回调
        if (frame->fdata->callback != NULL)
        {
            ret = frame->fdata->callback(frame->header.type, frame->from, frame->error_code);
        }
    }
}

void fprotocol_write(fprotocol_handler *handler, uint16_t node, uint8_t type, uint16_t index, void *data, uint16_t size)
{
    printf("size:%d\n",size);
    static uint8_t send_buff[FPROTOCOL_FRAME_DATA_SIZE];
    memcpy(send_buff, FRAME_HEAD, 4);
    fprotocol_frame *frame = handler->frame;
    frame->header.node = node;
    frame->header.type = type;
    frame->header.index = index;
    frame->header.data_size = size; // Data size 2Byte
    memcpy(send_buff + 4, &frame->header, sizeof(fprotocol_header));
    if (size)
    {
        memcpy(send_buff + 11, data, size);
    }
    uint16_t checksum = checksum16(send_buff, 11 + size);
    send_buff[11 + size] = checksum >> 8;
    send_buff[11 + size + 1] = checksum & 0xFF;
    handler->write(node, send_buff, 11 + size + 2);
}

fprotocol_handler *fprotocol_init(int32_t (*read)(int16_t, uint8_t *, int32_t), int32_t (*write)(int16_t, uint8_t *, int32_t))
{
    fprotocol_handler *handler = (fprotocol_handler *)malloc(sizeof(fprotocol_handler));
    if (handler == NULL)
    {
        return NULL; // 内存分配失败时返回 NULL
    }
    handler->rxbuff = (fring_buffer *)malloc(sizeof(fring_buffer));
    fring_clear(handler->rxbuff);
    handler->frame = (fprotocol_frame *)malloc(sizeof(fprotocol_frame));
    handler->frame->data = (uint8_t *)malloc(FPROTOCOL_FRAME_DATA_SIZE);
    handler->frame->recv_size = 0;
    handler->snode_count = 0;
    handler->read = read;
    handler->write = write;
    fring_init(handler->rxbuff, RING_BUFFER_SIZE);
    return handler; // 返回指针
}

void fprotocol_set_host_node(fprotocol_handler *handler, uint8_t node, fprotocol_get_index_info_t get_index_info)
{
    handler->node = node;
    handler->fprotocol_get_node_info = get_index_info;
}

void fring_init(fring_buffer *buffer, uint32_t size)
{
    if (buffer != NULL)
    {
        buffer->buf = (uint8_t *)malloc(size);
        if (buffer->buf != NULL)
        {
            buffer->bufSize = size;
            buffer->pIn = 0;
            buffer->pOut = 0;
        }
        else
        {
            free(buffer);
            buffer = NULL;
        }
    }
}

uint32_t fring_size(fring_buffer *p)
{
    if (p->pIn >= p->pOut)
    {
        return p->pIn - p->pOut;
    }
    else
    {
        return p->bufSize - (p->pOut - p->pIn);
    }
}

int32_t fring_put(fring_buffer *p, uint8_t *buf, uint32_t size)
{
    if (size + fring_size(p) >= p->bufSize)
    {
        return 0;
    }

    for (uint32_t i = 0; i < size; i++)
    {
        p->buf[p->pIn] = buf[i];
        p->pIn = (p->pIn + 1) % p->bufSize;
    }
    return size;
}

int32_t fring_get(fring_buffer *p, uint8_t *buf, uint32_t size)
{
    if (p->pIn == p->pOut)
    {
        return 0;
    }
    if (fring_size(p) < size)
    {
        return 0;
    }
    uint32_t i;
    for (i = 0; i < size; i++)
    {
        buf[i] = p->buf[p->pOut];
        p->pOut = (p->pOut + 1) % p->bufSize;

        if (p->pIn == p->pOut)
        {
            return i + 1;
        }
    }
    return i;
}

void fring_clear(fring_buffer *p)
{
    p->pIn = 0;
    p->pOut = 0;
}

uint16_t checksum16(const uint8_t *data, size_t length)
{
    uint32_t sum = 0;
    for (size_t i = 0; i < length; i++)
    {
        sum += data[i];
        if (sum & 0xFFFF0000)
        {
            sum = (sum & 0xFFFF) + (sum >> 16);
        }
    }
    return ~(sum & 0xFFFF);
}
