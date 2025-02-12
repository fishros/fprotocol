class CircularBuffer:
    def __init__(self, size):
        self.csize = size
        self.buffer = [None] * size
        self.head = 0
        self.tail = 0
        self.count = 0

    def put(self, data):
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
    def __init__(self, node=0, type=0, index=0):
        self.node = node
        self.type = type
        self.index = index

    @classmethod
    def from_bytes(cls, data):
        if len(data) != 5:
            raise ValueError("Invalid data length for FProtocolHeader")
        node = data[0] | (data[1] << 8)
        type = data[2]
        index = data[3] | (data[4] << 8)
        return cls(node, type, index)

    def to_bytes(self):
        return [
            self.node & 0xFF,
            (self.node >> 8) & 0xFF,
            self.type,
            self.index & 0xFF,
            (self.index >> 8) & 0xFF
        ]

    def __str__(self):
        return f"FProtocolHeader(node={self.node}, type={self.type}, index={self.index})"


class FProtocolType():
    SERVICE_REQUEST_WRITE = 0x01
    SERVICE_REQUEST_READ = 0x02  # Data 0 byte
    SERVICE_RESPONSE_WRITE = 0x03
    SERVICE_RESPONSE_READ = 0x04
    SERVICE_RESPONSE_ERROR = 0x05  # Data 2 bytes -> Error code
    TRANSPORT_DATA = 0x06
    HEART = 0x07
    MAX = 0x08
    
class FProtocol:
    FRAME_HEAD = [0x55,0xaa,0x55,0xaa]

    def __init__(self,read_callback,write_callback):
        self.slave_nodes = {}
        self.host_node = None
        self.host_node_proto = None
        self.read_callback = read_callback
        self.write_callback = write_callback
        self.rxbuff = CircularBuffer(1024)
        self.frame = {
            'recv_size': 0,
            'data': [],
            'header': FProtocol.FRAME_HEAD,
            'data_size': 0,
            'fdata': None,
            'error_code': 0
        }
    
    def add_slave_node(self, nodeid, nodeproto):
        self.slave_nodes[nodeid] = nodeproto

    def set_host_node(self, node, nodeproto):
        self.host_node = node
        self.host_node_proto = nodeproto

    def tick(self):
        if self.read_callback:
            data = self.read_callback()
            if data:
                self.rxbuff.put(data)
                # print("put",list(data),self.rxbuff.size())
        
        if self.frame['recv_size'] < 4:
            while self.frame['recv_size'] < 4:
                rdata = self.rxbuff.get(1)
                if rdata == -1:
                    break
                rdata = rdata[0]
                if rdata == FProtocol.FRAME_HEAD[self.frame['recv_size']]:
                    self.frame['data'].append(rdata)
                    self.frame['recv_size'] += 1
                elif rdata == FProtocol.FRAME_HEAD[0]:
                    self.frame['data'] = [rdata]
                    self.frame['recv_size'] = 1
                else:
                    self.frame['recv_size'] = 0
        
        if self.frame['recv_size'] >= 4 and self.frame['recv_size'] < 9:
            additional_data = self.rxbuff.get(9 - self.frame['recv_size'])
            if additional_data == -1:
                return
            self.frame['data'].extend(additional_data)
            self.frame['recv_size'] = len(self.frame['data'])
            if self.frame['recv_size'] != 9:
                return
            header = self.parse_header(self.frame['data'][4:9])
            self.frame['header']  = header

            if header.node in self.slave_nodes.keys():
                fdata = self.slave_nodes[self.frame['header'].node].get_index_data(self.frame['header'].index)
                self.frame['fdata'] = fdata
                if header.type == FProtocolType.SERVICE_RESPONSE_ERROR:
                    self.frame['data_size'] = 2
                elif header.type == FProtocolType.SERVICE_REQUEST_WRITE:
                    self.frame['data_size'] = 0
                elif header.type == FProtocolType.SERVICE_RESPONSE_READ:
                    self.frame['data_size'] = self.frame['fdata'].csize
                elif header.type == FProtocolType.TRANSPORT_DATA:
                    self.frame['data_size'] = self.frame['fdata'].csize
                else:
                    self.frame['recv_size'] = 0 # 重新接收
                # print(self.frame['recv_size'],self.frame['data_size'])
        
        if self.frame['recv_size'] >= 9 and self.frame['recv_size'] < (9 + self.frame['data_size']):
            additional_data = self.rxbuff.get(self.frame['data_size'])
            if additional_data == -1:
                return
            self.frame['data'].extend(additional_data)
            self.frame['recv_size'] = len(self.frame['data'])
            # print(self.frame['data'],additional_data,len(self.frame['data']))
        
        if self.frame['recv_size'] >= (9 + self.frame['data_size']):
            checksum_data = self.rxbuff.get(2) # checksum
            if checksum_data == -1:
                return
            # print("self.frame['recv_size']",self.frame['recv_size'],self.frame['data'],checksum_data)
            calculated_checksum = self.checksum16(self.frame['data'])
            received_checksum = checksum_data[0] << 8 | checksum_data[1]
            if calculated_checksum == received_checksum:
                if self.frame['header'].type == FProtocolType.SERVICE_RESPONSE_ERROR:
                    self.frame['error_code'] = self.frame['data'][9] << 8 | self.frame['data'][10]
                self.process_frame(self.frame)  

            self.frame['data'].clear()
            self.frame['recv_size'] = 0
            # print("self.frame['recv_size']",self.frame['recv_size'])
        if self.frame['recv_size'] > 1024:
            self.frame['recv_size'] = 0


    def process_frame(self,frame):
        header = frame['header']
        if header.node in self.slave_nodes.keys():
            if header.type == FProtocolType.SERVICE_RESPONSE_READ or header.type == FProtocolType.TRANSPORT_DATA:
                frame['fdata'].parse(bytes(frame['data'][9:]))
            if frame['fdata'].callback:
                frame['fdata'].callback(header.type,frame['fdata'],frame['error_code'])


    def fprotocol_write(self, node, type, index, data):
        frame = []
        frame.extend(FProtocol.FRAME_HEAD)
        frame.extend([
            node & 0xFF,
            (node >> 8) & 0xFF,
            type,
            index & 0xFF,
            (index >> 8) & 0xFF
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
    
    def write(self, data):
        result = self.buffer.put(data)
        if result == -1:
            print("Buffer is full, cannot write data")
        else:
            print(f"Data written to buffer: {data}")

import struct

class MainBProto:
    class can_frame_t:
        csize = 14
        cstruct = 'IBB8s'

        def __init__(self):
            self.arbitration_id = 0  # 4 bytes
            self.dlc = 0  # 1 byte
            self.rtr = 0  # 1 byte
            self.data = [0] * 8  # 8 bytes
            self.callback = None

        def parse(self, data):
            if len(data) != self.csize:
                raise ValueError(f"Data length must be {self.csize} bytes, got {len(data)} bytes.")
            unpacked = struct.unpack(self.cstruct, data)
            self.arbitration_id = unpacked[0]
            self.dlc = unpacked[1]
            self.rtr = unpacked[2]
            self.data = list(unpacked[3])

        def to_bytes(self):
            data_bytes = bytes(self.data[:8])  # Ensure the data is exactly 8 bytes
            packed = struct.pack(self.cstruct, self.arbitration_id, self.dlc, self.rtr, data_bytes)
            return packed

    def __init__(self):
        
        self.can_recv_frame = self.can_frame_t()
        self.can_send_frame = self.can_frame_t()

        self.index_table = {}
        self.index_table[0x0001] = self.can_recv_frame
        self.index_table[0x0002] = self.can_send_frame

        self.fprotocol_write = None

    def get_index_data(self,index):
        return self.index_table[index]

    def send_can_send_frame(self,fprotocol,type,node):
        fprotocol.fprotocol_write(node,type,0x0002,self.can_send_frame.to_bytes())


"""
A text based interface. For example use over serial ports like
"/dev/ttyS1" or "/dev/ttyUSB0" on Linux machines or "COM1" on Windows.
The interface is a simple implementation that has been used for
recording CAN traces.

See the interface documentation for the format being used.
"""

import io
import logging
import struct
from typing import Any, List, Optional, Tuple
from queue import Queue
import copy

from can import (
    BusABC,
    CanInitializationError,
    CanInterfaceNotImplementedError,
    CanOperationError,
    CanProtocol,
    CanTimeoutError,
    Message,
)
from can.typechecking import AutoDetectedConfig

logger = logging.getLogger("can.serial")

try:
    import serial
except ImportError:
    logger.warning(
        "You won't be able to use the serial can backend without "
        "the serial module installed!"
    )
    serial = None

try:
    from serial.tools.list_ports import comports as list_comports
except ImportError:
    # If unavailable on some platform, just return nothing
    def list_comports() -> List[Any]:
        return []


class SerialBus(BusABC):
    """
    Enable basic can communication over a serial device.

    .. note:: See :meth:`~_recv_internal` for some special semantics.

    """

    def __init__(
        self,
        channel: str,
        baudrate: int = 115200,
        timeout: float = 0.1,
        rtscts: bool = False,
        *args,
        **kwargs,
    ) -> None:
        """
        :param channel:
            The serial device to open. For example "/dev/ttyS1" or
            "/dev/ttyUSB0" on Linux or "COM1" on Windows systems.

        :param baudrate:
            Baud rate of the serial device in bit/s (default 115200).

            .. warning::
                Some serial port implementations don't care about the baudrate.

        :param timeout:
            Timeout for the serial device in seconds (default 0.1).

        :param rtscts:
            turn hardware handshake (RTS/CTS) on and off

        :raises ~can.exceptions.CanInitializationError:
            If the given parameters are invalid.
        :raises ~can.exceptions.CanInterfaceNotImplementedError:
            If the serial module is not installed.
        """
        if not serial:
            raise CanInterfaceNotImplementedError("the serial module is not installed")

        if not channel:
            raise TypeError("Must specify a serial port.")

        self.channel_info = f"Serial interface: {channel}"
        self._can_protocol = CanProtocol.CAN_20

        try:
            self.ser = serial.serial_for_url(
                channel, baudrate=baudrate, timeout=timeout, rtscts=rtscts
            )
        except ValueError as error:
            raise CanInitializationError(
                "could not create the serial device"
            ) from error        
        self.fprotocol = FProtocol(self.read_callback, self.write_callback)
        self.slave_node = MainBProto()
        self.slave_node.can_recv_frame.callback = self.callback_recv_can_frame
        self.fprotocol.add_slave_node(2, self.slave_node)
        self.msg_queue = Queue()
        super().__init__(channel, *args, **kwargs)

    def write_callback(self,data):
        # print("write_callback",list(data))
        self.ser.write(data)

    def read_callback(self):
        if self.ser.in_waiting > 0:
            data =self.ser.read(self.ser.in_waiting)
            # print(data)
            return data
        return None

    def callback_recv_can_frame(self,type,fdata,error_code):
        # print("callback_recv_can_frame")
        if type==FProtocolType.TRANSPORT_DATA:
            # print(fdata.to_bytes())
            msg = Message(
                timestamp=0,
                arbitration_id=fdata.arbitration_id,
                dlc=fdata.dlc,
                data=copy.deepcopy(fdata.data),
            )
            self.msg_queue.put(msg)
            # print(f"Arbitration ID: {fdata.arbitration_id}, DLC: {fdata.dlc}, RTR: {fdata.rtr}, Data: {fdata.data}")


    def shutdown(self) -> None:
        """
        Close the serial interface.
        """
        super().shutdown()
        self.ser.close()

    def send(self, msg: Message, timeout: Optional[float] = None) -> None:
        """
        Send a message over the serial device.

        :param msg:
            Message to send.

            .. note:: Flags like ``extended_id``, ``is_remote_frame`` and
                      ``is_error_frame`` will be ignored.

            .. note:: If the timestamp is a float value it will be converted
                      to an integer.

        :param timeout:
            This parameter will be ignored. The timeout value of the channel is
            used instead.

        """
        self.slave_node.can_send_frame.arbitration_id =  msg.arbitration_id
        self.slave_node.can_send_frame.dlc = msg.dlc
        self.slave_node.can_send_frame.rtr = 0
        self.slave_node.can_send_frame.data = msg.data
        self.slave_node.send_can_send_frame(self.fprotocol,FProtocolType.TRANSPORT_DATA,2) 


    def _recv_internal(
        self, timeout: Optional[float]
    ) -> Tuple[Optional[Message], bool]:
        """
        Read a message from the serial device.

        :param timeout:

            .. warning::
                This parameter will be ignored. The timeout value of the channel is used.

        :returns:
            Received message and :obj:`False` (because no filtering as taken place).

            .. warning::
                Flags like ``is_extended_id``, ``is_remote_frame`` and ``is_error_frame``
                will not be set over this function, the flags in the return
                message are the default values.
        """
        try:
            self.fprotocol.tick()
            if self.msg_queue.qsize() > 0:
                return self.msg_queue.get(), False
            return None, False
        except serial.SerialException as error:
            raise CanOperationError("could not read from serial") from error

    def fileno(self) -> int:
        try:
            return self.ser.fileno()
        except io.UnsupportedOperation:
            raise NotImplementedError(
                "fileno is not implemented using current CAN bus on this platform"
            ) from None
        except Exception as exception:
            raise CanOperationError("Cannot fetch fileno") from exception

    @staticmethod
    def _detect_available_configs() -> List[AutoDetectedConfig]:
        return [
            {"interface": "serial", "channel": port.device} for port in list_comports()
        ]
