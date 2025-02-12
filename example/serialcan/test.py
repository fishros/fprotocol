from fprotocol import FProtocol,FProtocolType
from mainbproto import MainBProto
import serial

# 打开串口
ser = serial.Serial('/dev/ttyS3', 115200, timeout=1)
# ser.write(bytearray([85, 170, 85, 170, 2, 0, 6, 2, 0, 2, 6, 0, 0, 8, 0, 35, 0, 32, 1, 255, 0, 0, 0, 252, 165]))
def read_callback():
    if ser.in_waiting > 0:
        data =ser.read(ser.in_waiting)
        # print(data)
        return data
    return None

def write_callback(data):
    print("write_callback",list(data))
    ser.write(data)


def callback_recv_can_frame(type,fdata,error_code):
    # print("callback_recv_can_frame")
    if type==FProtocolType.TRANSPORT_DATA:
        # print(fdata.to_bytes())
        print(f"Arbitration ID: {fdata.arbitration_id}, DLC: {fdata.dlc}, RTR: {fdata.rtr}, Data: {fdata.data}")

def main():
    protocol = FProtocol(read_callback, write_callback)
    
    # Add a dummy slave node for testing
    slave_node = MainBProto()
    slave_node.can_recv_frame.callback = callback_recv_can_frame
    protocol.add_slave_node(2, slave_node)
    
    slave_node.can_send_frame.arbitration_id = 0x602
    slave_node.can_send_frame.dlc = 8
    slave_node.can_send_frame.rtr = 0
    slave_node.can_send_frame.data = [0x23,0x00,0x20,0x01,0xFF,0x00,0x00,0x00]

    slave_node.send_can_send_frame(protocol,FProtocolType.TRANSPORT_DATA,2) 

    import time
    while True:
        time.sleep(0.1)
        protocol.tick()
        slave_node.can_send_frame.data[4]  = 0
        slave_node.send_can_send_frame(protocol,FProtocolType.TRANSPORT_DATA,2) 
        time.sleep(0.1)
        slave_node.can_send_frame.data[4]  = 255
        slave_node.send_can_send_frame(protocol,FProtocolType.TRANSPORT_DATA,2) 


if __name__ == "__main__":
    main()