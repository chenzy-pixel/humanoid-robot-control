# Python USB2CAN 接口

安装共享电机控制器与 SDK 兼容接口：

```bash
python -m pip install ./python
```

`lingzu` 提供电机协议和控制器；`pyusb2can` 提供同时收发两个 CAN 通道的底层接口。SOULDE v2.5 支持 CRC8、固定 17 字节包、通道过滤、分包、粘包及短写处理。单个高层控制器对应一个 CAN 通道。

```python
from lingzu.controller import LingzuMotorController

controller = LingzuMotorController(
    interface="soulde_usb2can", channel="/dev/USB2CAN0", can_channel=1)
controller.add_motor(1, "RS00")
try:
    print(controller.state_snapshot(1))
finally:
    controller.close()
```

Windows 可指定实际 COM 端口。PCAN、SocketCAN 和 SLCAN 另外安装 `python -m pip install './python[can]'`。电机型号、机械限位和运行模式应依据实际装配配置。ROS 控制节点的跟踪保护、软件标定和任务接口见 [程序控制说明](../docs/关节标定与程序控制.md)。

离线 CRC、通道、帧解析和 SDK 包装测试随仓库检查执行：

```bash
python -m unittest discover -s tests -p 'test_*.py'
```
