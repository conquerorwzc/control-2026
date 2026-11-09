# 髋关节 WP-ID（离线力矩回放初版）

本工具不控制硬件。实验不装气弹簧。需要真实固定机身闭链 MJCF，保留完整腿质量、轮子惯量及被动关节；禁用气弹簧，不得用示例机器人代替。MJCF 不含 actuator，使用 qfrc_applied 施加关节侧力矩。初始 keyframe 必须提供被动关节的正确闭链装配分支。

依赖：numpy、mujoco（新增离线依赖，因回放必须调用物理引擎；不影响固件构建）。

运行：python tools/hip_ident/fit_real2sim.py tools/hip_ident/config.json
所有路径相对执行目录。复制 config.template.json 为 config.json，补齐 null 后运行。

CSV：time_s，以及每个逻辑关节的 <joint>_q_rad、<joint>_qd_rad_s、<joint>_tau_cmd_nm。必须为无丢帧 500 Hz 数据。角度已标零和统一方向，力矩是保护限幅后、关节侧最终命令，不能用反馈力矩替代。开始前应稳定保持并记录一段稳态历史。温度、电流、帧号等额外列允许保留；录制侧另行检查重复反馈、饱和和故障。

参数顺序：4 armature (kg m²)、4 frictionloss (N m)、4 damping (N m s/rad)、4 torque scale、1 command delay (s)。均为关节侧参数。bounds 必须是17对实测有依据的上下界，禁止猜测。delay 为2 ms采样分辨率的等效命令延迟，非传感器延迟；辨识前先对齐反馈采样时间。已知反馈延迟不能让脚本错误地吸收到动力学中。

算法：可复现的有界随机搜索，包含边界中点基线；不保证全局最优或参数唯一。独立验证集不参与搜索。输出 fit.json、train_replay.csv、validation_replay.csv，可直接绘制实测与仿真叠加图。误差为每关节RMSE；此初版不自动判断相位是否源于延迟，不自动写回robot.yaml。差的参数边界可能导致仿真发散，工具会停止报告，不能静默吞掉模型问题。

重要限制：这是最终力矩命令回放，不是位置指令闭环回放；不能据此宣称已验证固件PD或内部电机控制器。需要额外使用留出的位置指令及实际控制律做闭环验收。真实数据缺失、模型缺失时不产出实车参数。尚待真实MJCF与实验数据端到端验收。
