import struct
from fprotocol import DynamicStruct,FProtocolType,BaseValue
class RobotProto:
    float3d_t_desc = [("x","f",0.0),("y","f",0.0),("z","f",0.0)]
    data_t_desc = [("temp","f",0.0),("hum","f",0.0),("ladc","B",0),("badc","B",0)]
    imu_t_desc = [("x","f",0.0),("y","f",0.0),("z","f",0.0),("acc_x","f",0.0),("acc_y","f",0.0),("acc_z","f",0.0),("gyro_x","f",0.0),("gyro_y","f",0.0),("gyro_z","f",0.0)]

    def __init__(self):
        self.index_table = {}
        self.ctrl_led = BaseValue(0, "B")
        self.index_table[0x0001] = self.ctrl_led
        self.ctrl_beep = BaseValue(0, "B")
        self.index_table[0x0002] = self.ctrl_beep
        self.ctrl_vel = DynamicStruct(self.float3d_t_desc)
        self.index_table[0x0003] = self.ctrl_vel
        self.sensor_odom = DynamicStruct(self.float3d_t_desc)
        self.index_table[0x0004] = self.sensor_odom
        self.sensor_board = DynamicStruct(self.data_t_desc)
        self.index_table[0x0005] = self.sensor_board
        self.sensor_imu = DynamicStruct(self.imu_t_desc)
        self.index_table[0x0006] = self.sensor_imu
        self.config_odom_hz = BaseValue(0, "B")
        self.index_table[0x0101] = self.config_odom_hz
        self.config_imu_hz = BaseValue(0, "B")
        self.index_table[0x0102] = self.config_imu_hz
        self.config_sensor_hz = BaseValue(0, "B")
        self.index_table[0x0103] = self.config_sensor_hz

    def get_index_data(self,index):
        return self.index_table[index]

    def __setattr__(self, name, value):
        # 如果属性是 BaseValue 且已存在，则更新其值而不是替换对象
        if hasattr(self, name) and isinstance(getattr(self, name), BaseValue):
            getattr(self, name).value = value
        else:
            object.__setattr__(self, name, value)

    def write_ctrl_led(self,fprotocol,type,node):
        bytes_data = self.ctrl_led.to_bytes()
        fprotocol.fprotocol_write(node,type,0x0001,bytes_data,len(bytes_data))

    def write_ctrl_beep(self,fprotocol,type,node):
        bytes_data = self.ctrl_beep.to_bytes()
        fprotocol.fprotocol_write(node,type,0x0002,bytes_data,len(bytes_data))

    def write_ctrl_vel(self,fprotocol,type,node):
        bytes_data = self.ctrl_vel.to_bytes()
        fprotocol.fprotocol_write(node,type,0x0003,bytes_data,len(bytes_data))

    def write_sensor_odom(self,fprotocol,type,node):
        bytes_data = self.sensor_odom.to_bytes()
        fprotocol.fprotocol_write(node,type,0x0004,bytes_data,len(bytes_data))

    def write_sensor_board(self,fprotocol,type,node):
        bytes_data = self.sensor_board.to_bytes()
        fprotocol.fprotocol_write(node,type,0x0005,bytes_data,len(bytes_data))

    def write_sensor_imu(self,fprotocol,type,node):
        bytes_data = self.sensor_imu.to_bytes()
        fprotocol.fprotocol_write(node,type,0x0006,bytes_data,len(bytes_data))

    def write_config_odom_hz(self,fprotocol,type,node):
        bytes_data = self.config_odom_hz.to_bytes()
        fprotocol.fprotocol_write(node,type,0x0101,bytes_data,len(bytes_data))

    def write_config_imu_hz(self,fprotocol,type,node):
        bytes_data = self.config_imu_hz.to_bytes()
        fprotocol.fprotocol_write(node,type,0x0102,bytes_data,len(bytes_data))

    def write_config_sensor_hz(self,fprotocol,type,node):
        bytes_data = self.config_sensor_hz.to_bytes()
        fprotocol.fprotocol_write(node,type,0x0103,bytes_data,len(bytes_data))

    def read_ctrl_led(self,fprotocol,node):
        fprotocol.fprotocol_write(node,FProtocolType.SERVICE_REQUEST_READ,0x0001,[],0)

    def read_ctrl_beep(self,fprotocol,node):
        fprotocol.fprotocol_write(node,FProtocolType.SERVICE_REQUEST_READ,0x0002,[],0)

    def read_ctrl_vel(self,fprotocol,node):
        fprotocol.fprotocol_write(node,FProtocolType.SERVICE_REQUEST_READ,0x0003,[],0)

    def read_sensor_odom(self,fprotocol,node):
        fprotocol.fprotocol_write(node,FProtocolType.SERVICE_REQUEST_READ,0x0004,[],0)

    def read_sensor_board(self,fprotocol,node):
        fprotocol.fprotocol_write(node,FProtocolType.SERVICE_REQUEST_READ,0x0005,[],0)

    def read_sensor_imu(self,fprotocol,node):
        fprotocol.fprotocol_write(node,FProtocolType.SERVICE_REQUEST_READ,0x0006,[],0)

    def read_config_odom_hz(self,fprotocol,node):
        fprotocol.fprotocol_write(node,FProtocolType.SERVICE_REQUEST_READ,0x0101,[],0)

    def read_config_imu_hz(self,fprotocol,node):
        fprotocol.fprotocol_write(node,FProtocolType.SERVICE_REQUEST_READ,0x0102,[],0)

    def read_config_sensor_hz(self,fprotocol,node):
        fprotocol.fprotocol_write(node,FProtocolType.SERVICE_REQUEST_READ,0x0103,[],0)