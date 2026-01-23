#include "fprotocol.h"
#ifdef ARDUINO
#include <Arduino.h>
#endif
#include <sys/time.h>
#include <time.h>
static uint8_t FRAME_HEAD[4] = {0x55, 0xAA, 0x55, 0xAA};

// #define DEBUG 1

static uint32_t fprotocol_now_ms()
{
#ifdef ARDUINO
    return millis();
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
#endif
}

static uint64_t fprotocol_utc_ms()
{
#ifdef ARDUINO
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)(tv.tv_usec / 1000ULL);
#else
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
#endif
}

void fprotocol_add_other_node(fprotocol_handler *handler, uint8_t node, fprotocol_get_index_info_t get_index_info)
{
    if (handler->other_node_count >= MAX_NODE)
    {
        return;
    }
    handler->other_node_index_table[handler->other_node_count] = node;
    handler->fprotocol_get_index_info_table[handler->other_node_count] = get_index_info;
    handler->other_node_count++;
}

fprotocol_data *fprotocol_get_other_node_data(fprotocol_handler *handler, uint8_t node, uint16_t index)
{
    if (handler->other_node_count == 0)
    {
        return NULL;
    }
    for (int i = 0; i < handler->other_node_count; i++)
    {

        if (handler->other_node_index_table[i] == node)
        {
            // printf("%d\n", __LINE__);
            // printf("%d:%p,%p\n",__LINE__,*handler,handler->fprotocol_get_index_info_table[i]);
            fprotocol_data *data = handler->fprotocol_get_index_info_table[i](index);
            // printf("%d\n", __LINE__);
            return data;
        }
    }
    return NULL;
}

void fprotocol_read_put(fprotocol_handler *handler, uint8_t *buf, uint32_t size)
{
    fring_put(handler->rxbuff, buf, size);
#ifdef DEBUG
    // printf("put data to ring_buffer_size: %lu %s\n", size, buf);
#endif
}

void fprotocol_tick(fprotocol_handler *handler)
{
    static uint8_t data[256];
    static uint8_t from = 0;
    uint8_t rdata;
    fprotocol_frame *frame = handler->frame;
    if (handler->read)
    {
        int16_t rsize = handler->read(from, data, 256);
        if (rsize > 0)
        {
            fring_put(handler->rxbuff, data, rsize);
#ifdef DEBUG
            // printf("put data to ring_buffer_size: %d\n", rsize);
#endif
        }
    }
#ifdef DEBUG
    // printf("fring_size: %ld frame->recv_size:%u\n",fring_size(handler->rxbuff), frame->recv_size);
#endif
    if (frame->recv_size < 4)
    {
#ifdef DEBUG
        // printf("frame->recv_size: %d\n", frame->recv_size);
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
#ifdef DEBUG
                // printf("fring_size(handler->rxbuff)=%d\n", fring_size(handler->rxbuff));
#endif
                break;
            }
        }
    }
    const uint16_t frame_header_size = sizeof(FRAME_HEAD) + sizeof(fprotocol_header);

    if (frame->recv_size >= 4 && frame->recv_size < frame_header_size)
    {
        frame->recv_size += fring_get(handler->rxbuff, frame->data + sizeof(FRAME_HEAD), sizeof(fprotocol_header));
        if (frame->recv_size != frame_header_size)
        {
            return;
        }
        else
        {
            memcpy(&frame->header, frame->data + sizeof(FRAME_HEAD), sizeof(fprotocol_header));
            frame->from = frame->header.from;
#ifdef DEBUG
            printf("Header Info - From: %02x, To: %02x, Type: %02x, Index: %d, DSize: %d \n",
                   frame->header.from, frame->header.to, frame->header.type, frame->header.index, frame->header.data_size);
            printf("%d self_node_id: %02x\n", __LINE__, handler->self_node_id);
#endif
            // 判断nodeid是否是自己的节点ID
            if (frame->header.to == handler->self_node_id || frame->header.to == BROADCAST_NODE_ID) // 只处理发给自己或广播
            {
                switch (frame->header.type)
                {
                case HEART_PONG:
                    frame->data_size = 0;
                    break;
                case SERVICE_RESPONSE_ERROR:
                    frame->data_size = frame->header.data_size; // Error code
                    break;
                case SERVICE_REQUEST_WRITE:
                case SERVICE_REQUEST_READ:
                case SERVICE_RESPONSE_READ:
                case TRANSPORT_DATA:
                case TIME_SYNC_REQ:
                case TIME_SYNC_RES:
                    frame->data_size = frame->header.data_size;
                    frame->fdata = (handler->fprotocol_get_node_info)(frame->header.index);
                    if (frame->fdata == NULL && frame->header.type < TIME_SYNC_REQ) // 无此索引且不是时间同步
                    {
#ifdef DEBUG
                        printf("No this index\n");
#endif
                        frame->data_size = 0;
                        frame->recv_size = 0;
                    }
                    break;
                default:
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

    if (frame->recv_size >= frame_header_size && frame->recv_size < (frame_header_size + frame->data_size))
    {
        frame->recv_size += fring_get(handler->rxbuff, frame->data + frame->recv_size, frame->data_size);
    }
    if (frame->recv_size >= frame_header_size + frame->data_size)
    {
        frame->recv_size += fring_get(handler->rxbuff, frame->data + frame->recv_size, 2);
        if (frame->recv_size == (frame_header_size + frame->data_size + 2))
        {
            uint16_t calculated_checksum = checksum16(frame->data, frame_header_size + frame->data_size);
            uint16_t received_checksum = (frame->data[frame_header_size + frame->data_size] << 8) | frame->data[frame_header_size + frame->data_size + 1];
            if (calculated_checksum == received_checksum)
            {
                fprotocol_req_deal(handler);
            }
            frame->recv_size = 0;
        }
    }

    if (frame->recv_size > (FPROTOCOL_FRAME_DATA_SIZE))
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
    const uint16_t frame_header_size = sizeof(FRAME_HEAD) + sizeof(fprotocol_header);
    // 从站
    if (frame->header.to == handler->self_node_id || frame->header.to == BROADCAST_NODE_ID)
    {
#ifdef DEBUG
        printf("Type: %02x\n", frame->header.type);
        printf("frame->header.from: %02x framr.fdata: %p\n", frame->header.from, frame->fdata);
#endif
        if (frame->header.type == HEART_PONG)
        {
            if (handler->heart_ping_callback != NULL)
            {
                handler->heart_ping_callback(frame->header.from);
            }
            return;
        }
        if (frame->header.type == TIME_SYNC_REQ)
        {
            uint64_t t0_ms = 0;
            if (frame->data_size >= 8)
            {
                memcpy(&t0_ms, frame->data + frame_header_size, 8);
            }
            uint64_t now_ms = fprotocol_utc_ms();
            uint8_t payload[16];
            memcpy(payload, &t0_ms, 8);
            memcpy(payload + 8, &now_ms, 8);
            fprotocol_write(handler, frame->header.from, TIME_SYNC_RES, 0, payload, 16, NULL);
        }
        else if (frame->header.type == TIME_SYNC_RES)
        {
            uint64_t t0_ms = 0, t1_ms = 0;
            if (frame->data_size >= 8)
            {
                memcpy(&t0_ms, frame->data + frame_header_size, 8);
            }
            if (frame->data_size >= 16)
            {
                memcpy(&t1_ms, frame->data + frame_header_size + 8, 8);
            }
            uint64_t t2_ms = fprotocol_utc_ms();
            uint64_t rtt = (t2_ms >= t0_ms) ? (t2_ms - t0_ms) : 0;
            uint64_t corrected_ms = t1_ms + (rtt / 2); // 对端时间 + 半RTT
            int64_t offset = (int64_t)corrected_ms - (int64_t)t2_ms;
            if (handler->time_sync_callback)
            {
                handler->time_sync_callback(frame->header.from, corrected_ms, offset, rtt);
            }
        }
        else
        {
            // 需要写入数据的情况
            if (frame->header.type == SERVICE_REQUEST_WRITE || frame->header.type == TRANSPORT_DATA)
            {
                fprotocol_unpack_struct(frame->data + frame_header_size, frame->fdata->data, frame->fdata->struct_desc); // 拷贝数据到指定节点数据体
            }
            // 回调
            if (frame->fdata && frame->fdata->callback != NULL)
            {
                ret = frame->fdata->callback(frame->header.type, frame->from, frame->error_code);
            }
            // 需要回复1 SERVICE_REQUEST_WRITE
            if (frame->header.type == SERVICE_REQUEST_WRITE)
            {
                if (ret == 0)
                {
                    fprotocol_write(handler, frame->header.from, SERVICE_RESPONSE_WRITE, frame->header.index, NULL, 0, NULL);
                }
                else
                {
                    fprotocol_write(handler, frame->header.from, SERVICE_RESPONSE_ERROR, frame->header.index, &ret, 2, NULL);
                }
            }
            // 需要回复2 SERVICE_REQUEST_WRITE
            if (frame->header.type == SERVICE_REQUEST_READ)
            {
                fprotocol_write(handler, frame->header.from, SERVICE_RESPONSE_READ, frame->header.index, frame->fdata->data, frame->fdata->data_size, frame->fdata->struct_desc);
            }
        }
    }
    else // 能到这里一定是主站了
    {
        // 需要写入数据的情况
        if (frame->header.type == SERVICE_RESPONSE_READ || frame->header.type == TRANSPORT_DATA)
        {
            memcpy(frame->fdata->data, frame->data + frame_header_size, frame->data_size); // 拷贝数据到指定节点数据体
        }
        // 回调
        if (frame->fdata->callback != NULL)
        {
            ret = frame->fdata->callback(frame->header.type, frame->from, frame->error_code);
        }
    }
}

size_t fprotocol_pack_struct(uint8_t *buffer, const void *data, const StructDescriptor *desc)
{
    size_t offset = 0;
    for (int i = 0; i < desc->field_count; ++i)
    {
        FieldDescriptor f = desc->fields[i];
        const uint8_t *field_ptr = (const uint8_t *)data + f.offset;
        uint16_t len = 0;

        if (f.size_field >= 0)
        {
            // 不定长数组
            const FieldDescriptor *size_fd = &desc->fields[f.size_field];
            const uint8_t *size_ptr = (const uint8_t *)data + size_fd->offset;
            len = (*size_ptr) + ((*(size_ptr + 1)) << 8);
             #ifdef DEBUG
                printf("len: %d, %d, %d\n", len, (*size_ptr << 8), *(size_ptr + 1));
            #endif
            // 单值字段
            switch (f.type)
            {
            case TYPE_INT8:
            case TYPE_UINT8:
                break;
            case TYPE_INT16:
            case TYPE_UINT16:
                len *= 2; 
                break;
            case TYPE_INT32:
            case TYPE_UINT32:
            case TYPE_FLOAT:
                len  *= 4; 
                break;
            default:
                break;
            }
        }
        else if (f.array_len > 0)
        {
            // 固定长度数组，需要乘以元素大小
            switch (f.type)
            {
            case TYPE_INT8:
            case TYPE_UINT8:
                len = f.array_len * 1;
                break;
            case TYPE_UINT16:
            case TYPE_INT16:
                len = f.array_len * 2;
                break;
            case TYPE_UINT32:
            case TYPE_INT32:
            case TYPE_FLOAT:
                len = f.array_len * 4;
                break;
            default:
                len = f.array_len;
                break;
            }
        }
        else
        {
            // 单值字段
            switch (f.type)
            {
            case TYPE_INT8:
            case TYPE_UINT8:
                len = 1;
                break;
            case TYPE_INT16:
            case TYPE_UINT16:
                len = 2;
                break;
            case TYPE_INT32:
            case TYPE_UINT32:
                len = 4;
                break;
            case TYPE_FLOAT:
                len = 4;
                break;
            default:
                break;
            }
        }

        memcpy(buffer + offset, field_ptr, len);
        offset += len;
        #ifdef DEBUG
            printf("offset: %d len: %d\n", offset, len);
        #endif
    }
    return offset;
}

size_t fprotocol_unpack_struct(const uint8_t *buffer, void *data, const StructDescriptor *desc)
{
#ifdef DEBUG
    printf("fprotocol_unpack_struct ");
#endif
    size_t offset = 0;
    for (int i = 0; i < desc->field_count; ++i)
    {
        FieldDescriptor f = desc->fields[i];
        uint8_t *field_ptr = (uint8_t *)data + f.offset;
        size_t len = 0;

        if (f.size_field >= 0)
        {
            // 不定长数组，先读前面的 length 字段
            const FieldDescriptor *size_fd = &desc->fields[f.size_field];
            const uint8_t *size_ptr = (const uint8_t *)data + size_fd->offset;
            len = *size_ptr;

        }
        else if (f.array_len > 0)
        {
            // 固定长度数组，需要乘以元素大小
            switch (f.type)
            {
            case TYPE_UINT8:
                len = f.array_len * 1;
                break;
            case TYPE_INT16:
            case TYPE_UINT16:
                len = f.array_len * 2;
                break;
            case TYPE_INT32:
            case TYPE_UINT32:
            case TYPE_FLOAT:
                len = f.array_len * 4;
                break;
            default:
                len = f.array_len;
                break;
            }
        }
        else
        {
            switch (f.type)
            {
            case TYPE_UINT8:
                len = 1;
                break;
            case TYPE_INT16:
            case TYPE_UINT16:
                len = 2;
                break;
            case TYPE_INT32:
            case TYPE_UINT32:
                len = 4;
                break;
            case TYPE_FLOAT:
                len = 4;
                break;
            default:
                break;
            }
        }

        memcpy(field_ptr, buffer + offset, len);
        offset += len;
    }
    return offset;
}

uint16_t fprotocol_write(fprotocol_handler *handler, uint8_t to, uint8_t type, uint16_t index, void *data, uint16_t size, const StructDescriptor *desc)
{
#ifdef DEBUG
    printf("fprotocol_write size:%d\n", size);
#endif
    static uint8_t send_buff[FPROTOCOL_FRAME_DATA_SIZE];
    memcpy(send_buff, FRAME_HEAD, 4);
    fprotocol_frame *frame = handler->frame;
    const uint16_t frame_header_size = sizeof(FRAME_HEAD) + sizeof(fprotocol_header);
    frame->header.from = handler->self_node_id;
    frame->header.to = to;
    frame->header.type = type;
    frame->header.index = index;
    if (size)
    {
        if (desc)
        {
            size = fprotocol_pack_struct(send_buff + frame_header_size, data, desc);
#ifdef DEBUG
            printf("fprotocol_pack_struct size:%d\n", size);
#endif
        }
        else
            memcpy(send_buff + frame_header_size, data, size); // Data
    }
    frame->header.data_size = size;
    memcpy(send_buff + 4, &frame->header, sizeof(fprotocol_header));
#ifdef DEBUG
    printf("data_size: %02x\n", frame->header.data_size);
#endif
    uint16_t checksum = checksum16(send_buff, frame_header_size + size);
    send_buff[frame_header_size + size] = checksum >> 8;
    send_buff[frame_header_size + size + 1] = checksum & 0xFF;
    return handler->write(to, send_buff, frame_header_size + size + 2);
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
    handler->other_node_count = 0;
    handler->read = read;
    handler->write = write;
    handler->heart_ping_callback = NULL;
    handler->time_sync_callback = NULL;
    fring_init(handler->rxbuff, RING_BUFFER_SIZE);
    return handler; // 返回指针
}

void fprotocol_set_self_node(fprotocol_handler *handler, uint8_t node, fprotocol_get_index_info_t get_index_info)
{
    handler->self_node_id = node;
    handler->fprotocol_get_node_info = get_index_info;
}

int8_t fprotocol_heart_ping(fprotocol_handler *handler, uint8_t target_node)
{
    return fprotocol_write(handler, target_node, HEART_PING, 0, NULL, 0, NULL) > 0;
}

int8_t fprotocol_set_heart_ping_callback(fprotocol_handler *handler, int8_t (*callback)(uint8_t node))
{
    if (handler == NULL || callback == NULL)
    {
        return -1; // Return error if handler or callback is NULL
    }
    handler->heart_ping_callback = callback;
    return 0; // Return success
}

int8_t fprotocol_time_sync(fprotocol_handler *handler, uint8_t target_node)
{
    uint64_t t0_ms = fprotocol_utc_ms();
    uint8_t payload[8];
    memcpy(payload, &t0_ms, 8);
    return fprotocol_write(handler, target_node, TIME_SYNC_REQ, 0, payload, 8, NULL) > 0;
}

int8_t fprotocol_set_time_sync_callback(fprotocol_handler *handler, int8_t (*callback)(uint8_t from, uint64_t corrected_utc_ms, int64_t offset_ms, uint64_t rtt_ms))
{
    if (handler == NULL || callback == NULL)
    {
        return -1;
    }
    handler->time_sync_callback = callback;
    return 0;
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
