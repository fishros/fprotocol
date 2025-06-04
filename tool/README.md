# FProtocol代码生成工具

使用方法：

编写 .fproto 文件，Example.fproto 参考格式如下:

```
# 数据结构定义部分
#数据结构名称,包含类型，名称,数据长度
canframe_t,uint32_t,arbitration_id,
canframe_t,uint8_t,dlc,
canframe_t,uint8_t,rtr,
canframe_t,uint8_t,data,8
---
# 地址 , 类型    ,  变量名,  ,是否回调
0x0001,canframe_t,can_recv_frame,1
0x0002,canframe_t,can_send_frame,1
```

可以直接使用的基础类型有：

```
'uint32_t',
'uint8_t',
'int32_t',
'int16_t',
'uint16_t',
'int8_t',
'float',
'double',
'char'
```

生成C代码：

```
python3 gencode_c.py ./Example.fproto ../example/circle_c/
```

生成Python代码：

```
python3 gencode_py.py ./Example.fproto ../example/circle_py/
```