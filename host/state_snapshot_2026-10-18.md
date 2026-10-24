# 机械臂状态快照 — 2026-10-18 11:54（Jetson 关机前）

- 机器人：192.168.31.127，在线，ADB 正常
- 状态：idle，电量 87%，无告警
- lift：0.44 m（`base_link → arm_base_link` z = 0.922）
- 臂姿态（抓取准备位）：`[0.186, 2.694, 2.15, -0.285, -1.065, 0]` rad
- 安全规则：任何向前/大范围运动前，先走侧摆A `[1.1, 1.2, 2.0, 0, -0.8, 0]`
  - 实测离柱 **6.1 cm**（lift=0 与 lift=0.44 相同；立柱与机械臂一起升降）
  - 余量 <6cm 直接中止
- 已部署到下板 `/home/Agi/jogd/`：`guard.py`（立柱 z 随 lift 平移）、`picksvc.py`（sideA 强制门）、`grasp2.py`（clear_column + 6cm 校验）
- `picksvc` 在机器人下板 `:9100` 运行；未启动抓取任务
- 当前头部扫描：0.46m 高、4.6×45.5cm 长条，不是手套；训练前需重新摆位/导航
- 下次开机顺序：
  1. 确认机器人 IP / ADB
  2. `tf2_echo base_link arm_base_link` 读 lift（lift=0 时 z=0.48267，升高后 =0.48267+lift）
  3. 同步 `/home/Agi/jogd/kin_state.json` 的 arm_base z
  4. 启动 `picksvc_start.sh`
  5. 空载验证 `grasp2.Grasper().clear_column(0.35)`，确认 6.1cm
  6. 再开始手套训练
