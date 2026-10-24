# robot_client — 新主机接入最小示例

`robot_client.py` 只依赖 Python 3 标准库，提供两种客户端：

- `RobotHTTP`：走 Jetson 关节桥（`:8796`）和指挥服务（`:8792`）
- `RobotMission`：直连机器人 Mission WebSocket（`:17890`）

## 只读自检（不会动机器人）

```bash
cd /opt/openhwlab/robot-viz/clients
python3 robot_client.py --jetson 127.0.0.1 --robot 192.168.1.100
```

输出类似：

```text
HTTP health: {'ok': True, 'offline': False, 'robot_online': True, ...}
HTTP joints: [5.4, 147.7, ...] lift 0.4393 online True
Mission info: {'battery': {...}, 'alarm': [], 'pos': {...}}
```

## 常用控制

```python
from robot_client import RobotHTTP, RobotMission

h = RobotHTTP("127.0.0.1")

# 小角度关节目标（阻塞）
h.set_joints([0.186, 2.694, 2.150, -0.285, -1.065, 0.0])

# 流式目标：持续发，最后 stop
h.jog([0.20, 2.60, 2.00, -0.30, -1.00, 0.0])
h.stop()

# 夹爪 0=闭，0.69=开
h.grip(0.69)

# 慢速模式
h.mode("slow")

# 高层动作（底盘/升降/头部/语音）
h.cmd({"action": "height", "value": 0.3})
h.cmd({"action": "head", "value": 1.0})

# 直连 Mission WS
m = RobotMission("192.168.1.100")
m.mission("robot_height_ctrl", {"value": 0.2})
m.mission("stop_all", {})
m.close()
```

## 注意

- 所有运动接口都没有鉴权，只在可信内网使用。
- 运动前确认周围安全、急停在手边；断线时请主动 `h.stop()` + `m.mission("stop_all")`。
- `RobotHTTP.set_joints` / `jog` 的角度单位是 **弧度**。
- 更多字段、限位和安全规则见 `../ROBOT_CONTROL_API.md`。
