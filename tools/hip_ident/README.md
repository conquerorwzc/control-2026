# 髋关节 WP-ID（离线力矩回放初版）

本工具不控制硬件。实验不装气弹簧。需要真实固定机身闭链 MJCF，保留完整腿质量、轮子惯量及被动关节；禁用气弹簧，不得用示例机器人代替。MJCF 不含 actuator，使用 qfrc_applied 施加关节侧力矩。初始 keyframe 必须提供被动关节的正确闭链装配分支。

依赖：numpy、mujoco（新增离线依赖，因回放必须调用物理引擎；不影响固件构建）。

运行：python tools/hip_ident/fit_real2sim.py tools/hip_ident/config.json
所有路径相对执行目录。复制 config.template.json 为 config.json，补齐 null 后运行。

CSV：time_s，以及每个逻辑关节的 <joint>_q_rad、<joint>_qd_rad_s、<joint>_tau_cmd_nm。必须为无丢帧 500 Hz 数据。角度已标零和统一方向，力矩是保护限幅后、关节侧最终命令，不能用反馈力矩替代。开始前应稳定保持并记录一段稳态历史。温度、电流、帧号等额外列允许保留；录制侧另行检查重复反馈、饱和和故障。

## VOFA+ 导出 CSV：离线校验与截取

`vofa_csv.py` 只读已导出的 CSV，不连接串口、不操作 VOFA+，只依赖 Python 3 标准库。在仓库根目录运行：

```powershell
# 仅检查整份录制；问题按类型报告次数和首个位置
python tools/hip_ident/vofa_csv.py vofa_train.csv

# 只转换指定区间；行号从 1 开始，不含表头，两端都包含
python tools/hip_ident/vofa_csv.py vofa_train.csv --start-row 201 --end-row 5200 --output train.csv
```

用另一份独立实验录制得到 `validation.csv`，将两份输出分别填入配置的 `train_csv` 和 `validation_csv`。选段应保留激励前的稳态历史；脚本不自动选段。导出至少需要两行，输出目录须已存在，不覆盖既有文件。可指定同样的行范围但省略 `--output` 做预检。检查失败退出非零；文件内可定位的问题即使在选段外也会报告，但不阻止干净选段的导出。只有区间内的行和内部相邻行之间的变化计入选段判定。

输入为带表头的 UTF-8 逗号 CSV（允许 BOM）。默认按名称查找 `CH0` 到 `CH23`，与文件内列的摆放顺序无关；多余列不参与校验。若导出表头被改名，用 `--channel-map channels.json` 明确提供 24 个实际表头名，数组位置就是通道号，每个名字只能对应唯一一列。下面示例适用于表头已改为协议字段名的文件，也展示了通道顺序：

```json
[
  "seq", "t_hi", "t_lo", "state", "mode", "flags",
  "left_front_q_rad", "left_front_qd_rad_s", "left_front_tau_cmd_nm", "left_front_tau_fb_nm",
  "left_rear_q_rad", "left_rear_qd_rad_s", "left_rear_tau_cmd_nm", "left_rear_tau_fb_nm",
  "right_front_q_rad", "right_front_qd_rad_s", "right_front_tau_cmd_nm", "right_front_tau_fb_nm",
  "right_rear_q_rad", "right_rear_qd_rad_s", "right_rear_tau_cmd_nm", "right_rear_tau_fb_nm",
  "q_ref0_rad", "q_ref1_rad"
]
```

- **时间与连续性**：只用固件 `(t_hi << 16) | t_lo`（32 位微秒计数），不使用 VOFA+ 的 `Time`、主机时间或行号推算。按原文件顺序检查 `seq` 模 65536 每拍加 1、时间每拍 2000 µs（按现有拟合器的 ±1 µs 浮点检查），允许正常回绕；重复、跳号、倒退/疑似重启、间隔异常及段内 `state`/`mode` 变化都会拒绝。不能从 CSV 判定首条记录之前、末条之后是否有丢帧，也不能对回绕与碰巧同时满足序号和时间连续的复位作区分。
- **真实执行**：每行都要求 `flags` bit 7 为 1，bit 5 驱动就绪、bit 6 遥控在线；空跑或执行中断的行不能进入实测选段。bit 4 为 1 时报告命令限幅，保留限幅后的 `tau_cmd_nm`；bits 8–9 不用于判断。bit 7 表示固件允许真实执行，并不单独证明电机实际出力。
- **反馈新鲜度**：每行必须四髋都由 `flags` bits 0–3 标为本拍新反馈，依次对应 `left_front`、`left_rear`、`right_front`、`right_rear`。不按数值是否重复猜测新鲜度。某髋没更新就拒绝整个选段，不删坏帧后拼接，不前向填充或插值；若固件反馈频率不足以形成连续四髋新鲜区间，需重新采集，而不是放宽检查。新鲜位也不代表四个反馈具有相同的精确采样时刻。
- **输出**：每条入选记录原样保留信号数值及协议字段，另加从首帧归零的 `time_s`（由实际固件时间差展开）；不重采样、不补帧、不用 `tau_fb_nm` 替代命令。协议列缺失、映射含歧义、选段内有坏行或 CSV 结构无法解析时不生成拟合输入。

本机测试（临时目录生成合成 CSV，不需要录制数据或拟合依赖）：

```powershell
python -B -m unittest discover -s tests -p "test_hip_ident_vofa_csv.py" -v
```

## 拟合与验收

参数顺序：4 armature (kg m²)、4 frictionloss (N m)、4 damping (N m s/rad)、4 torque scale、1 command delay (s)。均为关节侧参数。bounds 必须是17对实测有依据的上下界，禁止猜测。delay 为2 ms采样分辨率的等效命令延迟，非传感器延迟；辨识前先对齐反馈采样时间。已知反馈延迟不能让脚本错误地吸收到动力学中。

算法：可复现的有界随机搜索，包含边界中点基线；不保证全局最优或参数唯一。独立验证集不参与搜索。输出 fit.json、train_replay.csv、validation_replay.csv，可直接绘制实测与仿真叠加图。误差为每关节RMSE；此初版不自动判断相位是否源于延迟，不自动写回robot.yaml。差的参数边界可能导致仿真发散，工具会停止报告，不能静默吞掉模型问题。

重要限制：这是最终力矩命令回放，不是位置指令闭环回放；不能据此宣称已验证固件PD或内部电机控制器。需要额外使用留出的位置指令及实际控制律做闭环验收。真实数据缺失、模型缺失时不产出实车参数。尚待真实MJCF与实验数据端到端验收。
