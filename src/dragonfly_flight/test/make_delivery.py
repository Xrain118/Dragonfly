"""Collect the final evidence without copying large bags or changing raw runs."""
import hashlib
import json
from pathlib import Path
import shutil

root=Path('/home/ubuntu/Dragonfly')
destination=Path('/mnt/c/Users/LENOVO/Documents/ChatGPT/dragonfly/outputs/flight')
destination.mkdir(parents=True,exist_ok=True)
names=['effort_acceptance_'+str(i) for i in (2,3,4)]
metrics=[json.loads((root/'log'/n/'metrics.json').read_text()) for n in names]
assert all(m['passed'] for m in metrics)
models=[(root/'log'/n/'model.json').read_bytes() for n in names]
parameters=[(root/'log'/n/'px4_parameters.json').read_bytes() for n in names]
assert len(set(models))==1 and len(set(parameters))==1
keyboard=json.loads((root/'log/keyboard_all_pass/keyboard_result.json').read_text())
fault=json.loads((root/'log/dds_loss_verified/fault_result.json').read_text())
assert keyboard['passed'] and len(keyboard['jog_checks'])==20 and fault['passed']
text='''# Dragonfly 飞行与动臂验证记录（2026-09-30）

已实现 PX4 自由飞行、六路旋翼/三组双轴倾转、键盘单臂点动、真实 effort 动臂耦合、记录和自动验收。以下三轮均采用相同模型、控制器和 PX4 参数，禁用理想力辅助。代码在 `/home/ubuntu/Dragonfly/src/dragonfly_flight`。

## 三轮最终结果

| 运行 | 位置 RMS（m） | 姿态 RMS（°） | 动臂位置峰值（m） | 动臂姿态峰值（°） | 饱和最长时间（s） | 结果 |
|---|---:|---:|---:|---:|---:|---|
'''
for name,m in zip(names,metrics):
    text+=f"| {name} | {m['position_rms']:.4f} | {m['attitude_rms_deg']:.3f} | {m['arm_peak_position']:.4f} | {m['arm_peak_angle_deg']:.3f} | {m['longest_saturation_seconds']:.2f} | 通过 |\n"
    folder=destination/name;folder.mkdir(exist_ok=True)
    for filename in ('curves.png','metrics.json','model.json','controllers.yaml','px4_parameters.json','events.json'):
        shutil.copy2(root/'log'/name/filename,folder/filename)
text+='''
每轮均包括连续 30 s 固定臂悬停、X/Y 各 0.20 m 和 Z 0.10 m 平移，以及左右各一次“home→肩+5°→停留2s→home”。平移结束5s、动臂结束5s的误差均满足计划阈值。实际肩关节行程大于4.5°，关节和倾转力矩未持续饱和。记录轨迹的起落架包围盒采样检查通过；这不等同于全工作空间碰撞规划。

原始 CSV、ROS bag、PX4 ULog、参数和进程日志保存在 `/home/ubuntu/Dragonfly/log/<运行名称>/`，本目录保留曲线及小型参数/结果副本。之前的 `acceptance_*`、`keyboard_*` 调试目录不替代以上最终记录。

## 键盘与异常处理

真实伪终端运行键盘节点，连接 Gazebo effort 关节，完成两臂×五关节×正负两个方向共20项点动，逐项回 home。通过 Space 停臂、Esc / Ctrl+C 退出后继续飞行、心跳断开停臂，以及独立监控发现控制进程卡住后的仿真中止。测试记录为 `keyboard_result.json`。点动目标为±2°，1.7s后的实际读数约1.67～2.00°；最末关节仍存在小跟踪误差，未假装瞬时到位。

另冻结本次运行自己的 DDS Agent，验证 PX4 状态超过0.2仿真秒未更新时控制器中止，监控随后结束物理仿真，见 `fault_result.json`。键盘心跳阈值为0.5墙钟秒；飞行状态时间戳使用仿真时间。

±10°只是初始限位框，不是任意组合均可达。向起落架方向转底座约5°就可能接近碰撞；命令管理器用带3mm余量的包围盒拒绝这些路径，标准肩关节回放和±2°单关节点动路径通过检查。

## 物理验证与数值限制

自由落体及 X/Y/Z 单轴受力诊断通过。已知合力与整机动量变化一致，误差范数小于0.1N；继承的 Gazebo 重力约9.8m/s²，控制器重力补偿按9.81计算，因此理想模式有约0.046N竖向模型偏差。理想力只用于这些独立诊断，不用于最终飞行验收。

零重力、无旋翼外力和无理想力时，内部单臂动作产生约0.373°机体姿态响应。以下记录完整模型的线/角动量残差，未额外叠加机械臂扰动力。

| 步长（s） | 线动量峰值（kg·m/s） | 角动量峰值（kg·m²/s） | 机体转角峰值（°） |
|---:|---:|---:|---:|
'''
for step in ('.002','.001','.0005'):
    path=root/'log'/('momentum_verified_'+step)
    m=json.loads((path/'metrics.json').read_text())
    text+=f"| {m['step']} | {m['linear_peak']:.3e} | {m['angular_peak']:.3e} | {m['body_rotation_deg']:.6f} |\n"
    folder=destination/('momentum_'+step);folder.mkdir(exist_ok=True)
    for filename in ('metrics.json','momentum.csv'):shutil.copy2(path/filename,folder/filename)
text+='''
**未证明动量残差随减小步长单调收敛。** 机体响应跨步长基本一致，残差很小且无持续积累，但 RMS 存在数值波动。此项保留为引擎/采样精度问题，不能把整套物理验证描述为无条件全部通过；在做高级扰动补偿研究前应进一步查清。

## 保留与回归

原有35个机械臂源码文件与开工前快照逐字节一致；未提交改动保存在 `log/flight_baseline/source_before_flight.tar.gz` 和 `tracked.patch`。原 URDF、运动学、Gazebo 冒烟测试通过。最终单元/接口测试包含42个 pytest 用例，colcon 汇总含6个测试套件条目共48项，零失败。原生 x500 DDS 检查取得11条有效位置消息。

本机 PX4 v1.16.2、Agent v2.4.3、px4_msgs 固定提交，详见 `versions.yaml`。PX4 仅应用 opt-in 磁场坐标补丁，默认 x500 路径不变。动力学、惯量和执行器参数为仿真估值，不代表实物抗扰能力；没有加入动臂前馈、扰动观测器、NMPC 或学习补偿。

运行方法和按键表见 [启动说明](README.md)。
'''
(destination/'verification.md').write_text(text)
docs=root/'src/dragonfly_flight/docs';docs.mkdir(exist_ok=True)
(docs/'verification.md').write_text(text.replace('[启动说明](README.md)','[启动说明](../README.md)'))
for source,target in [(root/'src/dragonfly_flight/README.md','README.md'),
                      (root/'src/dragonfly_flight/config/versions.yaml','versions.yaml'),
                      (root/'src/dragonfly_flight/config/px4_mag_enu.patch','px4_mag_enu.patch'),
                      (root/'log/keyboard_all_pass/keyboard_result.json','keyboard_result.json'),
                      (root/'log/heartbeat_wall_verified/keyboard_result.json','heartbeat_wall_result.json'),
                      (root/'log/dds_loss_verified/fault_result.json','fault_result.json')]:
    shutil.copy2(source,destination/target)
(destination/'configuration_sha256.txt').write_text('model '+hashlib.sha256(models[0]).hexdigest()+'\nPX4 '+hashlib.sha256(parameters[0]).hexdigest()+'\n')
print(destination/'verification.md')
