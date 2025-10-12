import serial
import time

def main():
    # 打开串口 COM14，波特率 115200，超时1秒
    try:
        ser = serial.Serial('COM14', 115200, timeout=1)
    except serial.SerialException as e:
        print(f"串口打开失败: {e}")
        return

    print("串口已打开，开始收发数据...")

    count = 0
    try:
        while True:
            # 发送数据
            send_data = f'hello esp32 {count}\n'
            ser.write(send_data.encode('utf-8'))
            print(f"发送: {send_data.strip()}")

            # 接收二进制数据（最多一次读取64字节）
            recv_data = ser.read(64)
            if recv_data:
                print(f"接收({len(recv_data)} bytes): {recv_data.hex(' ')}")

            count += 1
            time.sleep(1)
    except KeyboardInterrupt:
        print("用户中断，退出程序。")
    finally:
        ser.close()
        print("串口已关闭。")

if __name__ == '__main__':
    main()
