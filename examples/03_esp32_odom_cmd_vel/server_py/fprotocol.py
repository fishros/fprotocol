import logging
logging.basicConfig(
    level=logging.WARN,
    format='%(levelname)s:%(filename)s:%(funcName)s:%(lineno)d->%(message)s',
)
logger = logging.getLogger(__name__)


import struct

class DynamicStruct:
    def __init__(self, field_defs):
        """
        field_defs: list of tuples (field_name, format_char)
        Example: [("id", "H"), ("payload", "1024s")]
        """
        self.fields = field_defs
        self.format = ''.join(fmt for _, fmt,_ in field_defs)
        self.size = struct.calcsize(self.format)
        self.values = {name: default_value for name, _,default_value in field_defs}
        self.callback = None


    def parse(self, data: bytes):
        index = 0
        next_array_size = 0
        total_len = len(data)
        
        for field in self.fields:
            field_name, fmt, _ = field
            field_size = struct.calcsize(fmt)

            
            # 原有解析逻辑
            if field_name.startswith("_"):
                self.values[field_name] = struct.unpack(fmt, data[index:index+field_size])[0]
                next_array_size = self.values[field_name]
                index += field_size
            else:
                if next_array_size:
                    fmt_without_num = fmt[-1]  # Get the format type (like 's', 'H', etc.)
                    dynamic_fmt = f"{next_array_size}{fmt_without_num}"

                    logger.debug(f'field_name {field_name}')
                    logger.debug(f'next_array_size {next_array_size}')
                    logger.debug(fmt)
                    logger.debug(data[index:index+field_size])

                    self.values[field_name] = struct.unpack(dynamic_fmt, data[index:index+next_array_size])[0]
                    index+= next_array_size
                    next_array_size = 0
                else:
                    
                    logger.debug(f'field_name {field_name}')
                    logger.debug(fmt)
                    logger.debug(data[index:index+field_size])

                    self.values[field_name] = struct.unpack(fmt, data[index:index+field_size])[0]
                    index+= field_size

    def to_bytes(self) -> bytes:
        packed = bytearray()
        next_array_size = 0
        for field_name, fmt,_ in self.fields:
            val = self.values.get(field_name)
            
            if field_name.startswith("_"):
                # 普通字段，打包并记录大小供下个字段使用
                # print("field_name",field_name,fmt,val)
                packed += struct.pack(fmt, val)
                next_array_size = val
            else:
                if next_array_size:
                    # 使用前面的字段长度生成动态格式
                    fmt_type = fmt[-1]
                    dynamic_fmt = f"{next_array_size}{fmt_type}"
                    if fmt_type == "s":
                        if isinstance(val, str):
                            val = val.encode()
                        val = val.ljust(next_array_size, b'\x00')
                    packed += struct.pack(dynamic_fmt, val)
                    next_array_size = 0
                else:
                    # 非变长字段，正常处理
                    if fmt.endswith("s") and isinstance(val, str):
                        val = val.encode()
                        val = val.ljust(int(fmt[:-1]), b'\x00')
                    packed += struct.pack(fmt, val)
        return bytes(packed)


    def __getattr__(self, name):
        return self.values[name]

    def __setattr__(self, name, value):
        if name in ("fields", "format", "size", "values"):
            super().__setattr__(name, value)
        else:
            self.values[name] = value

    def __str__(self):
        field_strs = []
        for field_name, _, _ in self.fields:
            value = self.values.get(field_name)
            field_strs.append(f"{field_name}={value}")
        return f"DynamicStruct({', '.join(field_strs)})"
    
    def to_json(self):
        json_data = {}
        for field_name, _, _ in self.fields:
            value = self.values.get(field_name)
            # Convert bytes to hex string if value is bytes
            if isinstance(value, bytes):
                value = value.hex()
            json_data[field_name] = value
        return json_data


class CircularBuffer:
    def __init__(self, size):
        self.csize = size
        self.buffer = [None] * size
        self.head = 0
        self.tail = 0
        self.count = 0

    def put(self, data):
        logger.debug(f"Buffer size: {self.csize}, Current count: {self.count}, Data length: {len(data)}")
        if len(data) > self.csize - self.count:
            return -1  # Not enough space in buffer
        for byte in data:
            self.buffer[self.tail] = byte
            self.tail = (self.tail + 1) % self.csize
            self.count += 1
        return 0  # Success

    def get(self, n=1):
        if self.count == 0:
            return -1  # Buffer is empty
        if n > self.count:
            return -1  # Not enough data
        data = []
        for _ in range(n):
            data.append(self.buffer[self.head])
            self.head = (self.head + 1) % self.csize
            self.count -= 1
        return data

    def size(self):
        return self.count

    def clear(self):
        self.head = 0
        self.tail = 0
        self.count = 0
        self.buffer = [None] * self.csize

class FProtocolHeader:
    def __init__(self, node=0, type=0, index=0, data_size=0):
        self.node = node
        self.type = type
        self.index = index
        self.data_size = data_size

    @classmethod
    def from_bytes(cls, data):
        if len(data) != 7:
            raise ValueError("Invalid data length for FProtocolHeader")
        node = data[0] | (data[1] << 8)
        type = data[2]
        index = data[3] | (data[4] << 8)
        data_size = data[5] | (data[6] << 8)
        return cls(node, type, index, data_size)

    def to_bytes(self):
        return [
            self.node & 0xFF,
            (self.node >> 8) & 0xFF,
            self.type,
            self.index & 0xFF,
            (self.index >> 8) & 0xFF,
            self.data_size & 0xFF,
            (self.data_size >> 8) & 0xFF
        ]

    def __str__(self):
        return f"FProtocolHeader(node={self.node}, type={self.type}, index={self.index}, data_size={self.data_size})"


class FProtocolType():
    SERVICE_REQUEST_WRITE = 0x01
    SERVICE_REQUEST_READ = 0x02  # Data 0 byte
    SERVICE_RESPONSE_WRITE = 0x03
    SERVICE_RESPONSE_READ = 0x04
    SERVICE_RESPONSE_ERROR = 0x05  # Data 2 bytes -> Error code
    TRANSPORT_DATA = 0x06
    HEART_PING = 0x07
    HEART_PONG = 0x08
    MAX = 0x08
    
class FProtocol:
    FRAME_HEAD = [0x55,0xaa,0x55,0xaa]

    def __init__(self, read_callback, write_callback):
        self.other_nodes = {}  # 其他节点的协议对象
        self.self_node_id = None  # 自己的节点ID
        self.self_node_proto = None  # 自己的协议对象
        self.read_callback = read_callback
        self.write_callback = write_callback
        self.rxbuff = CircularBuffer(20480)
        self.frame = {
            'recv_size': 0,
            'data': [],
            'header': FProtocol.FRAME_HEAD,
            'data_size': 0,
            'fdata': None,
            'error_code': 0
        }
        # 心跳回调函数
        self.heartbeat_callback = None
    
    def add_other_node(self, node_id, node_proto):
        """添加其他节点"""
        self.other_nodes[node_id] = node_proto

    def set_self_node(self, node_id, node_proto):
        """设置自己的节点ID和协议对象"""
        self.self_node_id = node_id
        self.self_node_proto = node_proto
    
    def set_heartbeat_callback(self, callback):
        """设置心跳回调函数"""
        self.heartbeat_callback = callback
    
    def send_heartbeat(self, target_node=None):
        """发送心跳包，默认使用自身节点ID"""
        if target_node is None:
            target_node = self.self_node_id
        return self.fprotocol_write(target_node, FProtocolType.HEART_PING, 0, [])

    def tick(self):
        # 批量读取数据，提高性能
        if self.read_callback:
            data = self.read_callback()
            if data:
                self.rxbuff.put(data)
                logging.debug(f"put {list(data)} {self.rxbuff.size()}")
        
        # 批量处理多个数据包
        processed_packets = 0
        max_packets_per_tick = 200  # 每次tick最多处理20个包
        
        while processed_packets < max_packets_per_tick and self.rxbuff.size() > 0:
            # 尝试处理一个完整的数据包
            if self._process_single_packet():
                processed_packets += 1
            else:
                # 如果没有完整包可处理，退出循环
                break
    
    def _process_single_packet(self):
        """处理单个数据包，返回True表示成功处理了一个包"""
        # 帧头检测
        if self.frame['recv_size'] < 4:
            available_data = self.rxbuff.get(min(4 - self.frame['recv_size'], self.rxbuff.size()))
            if available_data == -1:
                return False
            
            for rdata in available_data:
                if rdata == FProtocol.FRAME_HEAD[self.frame['recv_size']]:
                    self.frame['data'].append(rdata)
                    self.frame['recv_size'] += 1
                elif rdata == FProtocol.FRAME_HEAD[0]:
                    self.frame['data'] = [rdata]
                    self.frame['recv_size'] = 1
                else:
                    self.frame['recv_size'] = 0
                    self.frame['data'] = []
        
        # 读取头部信息
        if self.frame['recv_size'] >= 4 and self.frame['recv_size'] < 11:
            additional_data = self.rxbuff.get(11 - self.frame['recv_size'])
            if additional_data == -1:
                return False
            self.frame['data'].extend(additional_data)
            self.frame['recv_size'] = len(self.frame['data'])
            if self.frame['recv_size'] != 11:
                return False
            header = self.parse_header(self.frame['data'][4:11])
            self.frame['header'] = header
            logger.debug(f"header={header}")
            
            if header.node == self.self_node_id:
                logger.debug(f"Recv data from self node {header.node}")
            elif header.node in self.other_nodes.keys():
                if header.type == FProtocolType.HEART_PING:
                    self.frame['data_size'] = 0
                    self.frame['fdata'] = None
                    logger.debug(f"Recv data from node {self.frame['header'].node} data_size={self.frame['header'].data_size}")
                else:
                    if header.type == FProtocolType.SERVICE_RESPONSE_ERROR:
                        self.frame['data_size'] = self.frame['header'].data_size  # 2
                    elif header.type == FProtocolType.SERVICE_REQUEST_WRITE:
                        self.frame['data_size'] = self.frame['header'].data_size # 0
                    elif header.type == FProtocolType.SERVICE_RESPONSE_READ:
                        self.frame['data_size'] = self.frame['header'].data_size 
                    elif header.type == FProtocolType.TRANSPORT_DATA:
                        self.frame['data_size'] = self.frame['header'].data_size 
                    else:
                        self.frame['recv_size'] = 0 # 重新接收
            else:
                self.frame['recv_size'] = 0
                return False
                
        # 读取数据部分
        if self.frame['recv_size'] >= 11 and self.frame['recv_size'] < (11 + self.frame['data_size']):
            additional_data = self.rxbuff.get(self.frame['data_size'])
            if additional_data == -1:
                return False
            self.frame['data'].extend(additional_data)
            self.frame['recv_size'] = len(self.frame['data'])
            logger.debug(f"Recv data from node {self.frame['header'].node} data_size={self.frame['data_size']}")
        
        # 读取校验和并处理完整包
        if self.frame['recv_size'] >= (11 + self.frame['data_size']):
            checksum_data = self.rxbuff.get(2) # checksum
            if checksum_data == -1:
                return False
            
            calculated_checksum = self.checksum16(self.frame['data'])
            received_checksum = checksum_data[0] << 8 | checksum_data[1]
            if calculated_checksum == received_checksum:
                if self.frame['header'].type == FProtocolType.SERVICE_RESPONSE_ERROR:
                    self.frame['error_code'] = self.frame['data'][11] << 8 | self.frame['data'][12]
                self.process_frame(self.frame)  
                # 成功处理了一个包
                self.frame['data'].clear()
                self.frame['recv_size'] = 0
                return True
            else:
                # 校验失败，重置
                self.frame['recv_size'] = 0
                return False
                
        if self.frame['recv_size'] > 1024:
            self.frame['recv_size'] = 0
            
        return False


    def process_frame(self, frame):
        header = frame['header']
        fdata = None
        
        # 处理心跳
        if header.type == FProtocolType.HEART_PING and header.node in self.other_nodes.keys():
            # 发送心跳响应
            if self.heartbeat_callback:
                self.heartbeat_callback(header.type, header.node, 0)
            self.fprotocol_write(header.node, FProtocolType.HEART_PONG, 0, [])
            print(f"[Heartbeat] 收到来自节点 {header.node} 的心跳，已回复")
        
        # 处理其他类型的消息
        elif header.node in self.other_nodes.keys():
            if header.type == FProtocolType.SERVICE_RESPONSE_READ or header.type == FProtocolType.TRANSPORT_DATA:
                fdata = self.other_nodes[self.frame['header'].node].get_index_data(self.frame['header'].index)
                fdata.parse(bytes(frame['data'][11:]))

            if fdata and fdata.callback:
                fdata.callback(header.type, header.node, frame['error_code'])


    def fprotocol_write(self, node, type, index, data, data_size=0):
        frame = []
        frame.extend(FProtocol.FRAME_HEAD)
        frame.extend([
            node & 0xFF,
            (node >> 8) & 0xFF,
            type,
            index & 0xFF,
            (index >> 8) & 0xFF,
            data_size & 0xFF,
            (data_size >> 8) & 0xFF
        ])
        if data:
            frame.extend(data)
        checksum = self.checksum16(frame)
        frame.append(checksum >> 8 & 0xFF)
        frame.append(checksum & 0xFF)
        self.write_callback(bytes(frame))

    def checksum16(self,data):
        sum_val = 0
        for byte in data:
            sum_val += byte
            # Handle carry (keep sum within 16 bits)
            if sum_val & 0xFFFF0000:
                sum_val = (sum_val & 0xFFFF) + (sum_val >> 16)
        # Return one's complement of the lower 16 bits
        return (~sum_val & 0xFFFF)

    def parse_header(self,data):
        head = FProtocolHeader.from_bytes(data)
        # print("parse_header",head,head.to_bytes())
        return head
    
    def read_put(self, data):
        result = self.rxbuff.put(data)
        if result == -1:
            logger.debug("Buffer is full, cannot write data")
        else:
            logger.debug(f"Data written to buffer: {data}")
            self.tick()
