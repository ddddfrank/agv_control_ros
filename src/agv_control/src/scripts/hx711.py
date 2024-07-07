import serial  
import struct  
import math  
  
# 初始化串口  
ser = serial.Serial('COM4', 9600, timeout=1)  # 假设波特率为9600  
  
def read_and_display_u32():  
    while True:  
        if ser.in_waiting >= 5:  # 等待至少5个字节（0xff + 4个u32字节）  
            data = ser.read(5)  # 读取5个字节  
            if data[0] == 0xff:  # 检查是否以0xff开头  
                # 去掉头部，并将剩余部分转换为32位整数  
                u32_value = struct.unpack('<I', data[1:5])[0]  
                  
         
                if u32_value == 0:  
                    print(f"Received value is 0")  
                else:  
                    # 假设x是u32_value（或者你可以将x设置为其他值）  
                    x = u32_value-1000000000  
                    # 计算y的值（注意这里使用math.exp来计算e的指数）  
                    y = (5.3960282588188) * math.exp((0.00000008125591150515) * x) 
                    # 取整数部分（向下取整）  
                    y_integer = int(y)  
                    print(f"Computed integer value: {y_integer}")  
  
# 开始读取和显示数据  
read_and_display_u32()
