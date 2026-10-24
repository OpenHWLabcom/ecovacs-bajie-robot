# 科沃思八界机器人（Ecovacs BaJie）· 开源

**当前开源型号：科沃思八界（Ecovacs BaJie）** —— 6 轴协作机械臂 + 升降立柱 + 两指夹爪。

一套围绕该型号的开源控制与数据闭环：
3D 真机仿真、可视化控制台、视觉/语音交互、抓取策略、数据采集标注、训练分析与测试验收。

> 智能硬件开放实验室研发 · MIT License
> 官网：https://www.openhwlab.com/robot.html
> GitHub：https://github.com/OpenHWLabcom/ecovacs-bajie-robot

---

## 目录结构

```
host/                 上位机（Jetson / Linux）控制台与控制服务
  robot_home.html     子系统导航页
  index.html          主控制台（实时看板 / 关节执行 / 升降 / 夹爪）
  arm3d.html          3D 机械臂（真机网格 / 拖拽示教 / 定点头 / 限位保护）
  grip.html           夹爪控制
  voice.html          语音控制
  cmd.html            指挥系统
  data_viewer.html    训练数据综合分析台
  headfix_collect.html 定点头数据采集
  headfix_limits.html 限位数据库
  annotate.html       抓取物 / 存放位置标注
  test_plan.html      测试计划与验收清单
  dance_editor.html   舞蹈 / 动作编辑器
  joints_bridge.py    关节状态桥（:8796，adb→chroot→ssh→ROS2）
  mode_svc.py         真机 / 本地模式服务（:8795，掉线自动切换）
  camfeed.py          双路相机流（:8799 / :8800）
  guard.py / kin.py   碰撞限位与运动学
  meshes/             URDF 网格（STL）
  vendor/             three.js 与 STL 加载器
  clients/            零依赖 Python 客户端（HTTP / Mission WS）
lower/jogd/           机器人下位机脚本（ROS2 Humble）
docs/                 运动规则 / 抓取规则 / 控制 API / 测试计划
```

## 硬件与系统（参考搭建）

- 机械臂：6 轴 + 升降立柱（0–0.44 m）+ 两指夹爪（0–0.69）
- 上位机：Jetson / Ubuntu 20.04+，Python 3，Node 可选
- 机器人上位板：Android + Ubuntu chroot（`/linux`），开放 ADB 5555
- 下位机：ROS 2 Humble，`/joint_states`、`/arm_controller/joint_trajectory`
- 相机：头部 RGBD + 腕部相机（MJPEG 推流）

链路：`Jetson → adb → chroot /linux → ssh Agi@<lower> → ROS 2`

## 快速开始

```bash
# 1) 上位机控制台
cd host
# 按实际环境修改 serve.sh 里的 DIR / ADB 路径，以及 ROBOT_HOST / ROBOT_PW
bash serve.sh
# 打开 http://<jetson-ip>:8790/robot_home.html

# 2) 只启动静态页（没有机器人也能看 3D / 页面）
python3 -m http.server 8790
```

常用环境变量：

| 变量 | 默认 | 说明 |
|---|---|---|
| `ROBOT_HOST` | `192.168.1.100` | 机器人上位板 IP（adb） |
| `ROBOT_PW` | `robot` | 下位机 ssh 密码（**请务必修改**） |
| `LOWER_IP` | `10.0.0.2` | 下位机内网 IP |
| `ADB` | `adb` | adb 可执行文件路径 |

下位机：

```bash
scp -r lower/jogd Agi@<lower>:/home/Agi/
ssh Agi@<lower> 'python3 /home/Agi/jogd/jtstream.py'
```

## 安全提示（重要）

机械臂是**有力量的真实设备**。首次运行前：

1. 清空工作范围，急停放在手边；
2. 先空载、慢速（`mode=slow`）验证；
3. 任何大范围姿态切换先走安全侧摆位，确认离立柱 ≥ 6 cm；
4. 抓取只允许从上往下，预抓取高度 ≥ 桌面 + 12 cm；
5. 详见 `docs/MOTION_RULES.md` 与 `docs/GRASP_RULES.md`。

## 许可

MIT License，详见 `LICENSE`。仅供学习与研究，改造与运行风险自负。
