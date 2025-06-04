from exampleptoto import ExampleProto
from fprotocol import *

handler = None

def read_callback():
    return None

def write_callback(data):
    print("write_callback len=%d -> ",len(data),list(data))
    handler.read_put(data)

handler = FProtocol(read_callback,write_callback)
    

proto = ExampleProto()

def can_sen_callback(type,from_node,error_code):
    print(f"can_sen_callback type={type} from_node={from_node} error_code={error_code}")

proto.can_send_frame.callback = can_sen_callback

handler.add_slave_node(0x0001,proto)
proto.send_can_send_frame(handler,FProtocolType.TRANSPORT_DATA,0x0001)
# proto.tick()
