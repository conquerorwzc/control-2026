# 1127 电池版 · 无簧台架第 1 项

## 本次操作

1. 底盘架空并固定，保留完整左腿和轮子，先不装气弹簧。把左腿放到行程内部、中等腿长附近（约 260 mm）。本次只使能左侧两个关节电机，轮电机不参与。
2. 在 CLion 打开固件仓库，选择默认 Debug 配置的 `control-2026.elf`，自行编译、烧录。当前任务参数在 `UserApp/robot/motor_sysid/robot_config.h`。
3. 上电时至少一个拨杆不在上档。看到 VOFA 的 JustFloat 25 通道刷新后开始记录，然后将两拨杆拨到双上，拨轮可放在任意位置。遥控 UART5，当前工程 VOFA 使用 UART7、921600 baud。
4. 双上的瞬间锁存左前、左后髋的实测角。左后髋保持，左前髋从当前位置以 ±0.05 rad、0.5–5 Hz 扫频。1 秒保持、20 秒扫频、1 秒恢复。完成自动停输出，持续双上只运行一次。中途拨离双上即停止，再次拨到双上可重做。
5. 保留前后几秒待机，导出整份原始 CSV 发回即可。表头 I0–I24 或 CH0–CH24 均可，直接保留原始列、时间和帧序号。后续由分析端提取有效片段，检查掉帧、两髋反馈、饱和和跟踪。

先完成这一次无簧试验并核对 CSV，再安排下一项。CAN 反馈力矩按 1 倍近似实际关节力矩。

## 离线分析参考（采集者只需交付 CSV）

在固件仓库根目录执行。把示例输入替换成自己的真实录制和解码后实际文件名。

```powershell
$py = 'D:/Anaconda3/envs/mujoco/python.exe'
$kit = 'D:/Code-RM/github/build/nospring-kit'
$cad = 'C:/Users/Dai/Desktop/wheel_leg_1127_battery'
& $py tools/hip_ident/bench_model.py $cad "$kit/bench_removed.xml" --spring removed
& $py tools/hip_ident/extract_bench.py raw_left_sweep.csv decoded_left_sweep --spring removed
& $py tools/hip_ident/extract_bench.py raw_left_grid.csv decoded_left_grid --spring removed
```

每个原始文件单独解码。输出目录的 `index.json` 列出导出片段、缺帧边界和拒绝段；断开的试次不拼接。网格片段可分别汇总，检查接近方向和实际有效点，不以总行数代替完整网格。对动态拟合只选头尾保持均完整、反馈新鲜、激励有效的录制，出现缺帧可补录这一试次。

```powershell
# $files 填入同一侧、同一装簧状态的相关解码 CSV；可同时包含多个工作点及多档速度
$files = @(Get-ChildItem -Path 'decoded_left_*' -Filter 'rows_*.csv' -Recurse | ForEach-Object FullName)
& $py tools/hip_ident/bench_data.py left @files --spring removed --output left_removed.json
Copy-Item tools/hip_ident/bench_fit.template.json "$kit/left_fit.json"
# 在 left_fit.json 填实测坐标映射和已取得的运动摩擦结果，填写每轴候选附加惯量、训练与独立留出 CSV 路径
& $py tools/hip_ident/bench_static.py "$kit/left_fit.json" left_removed.json left_removed_static.csv
& $py tools/hip_ident/fit_real2sim.py "$kit/left_fit.json"
```

`bench_data.py` 输出静态平台通过数、两髋力矩与波动、起动增量、正反稳速配对覆盖度、扫频角度幅值比和相位、指令发布到 CAN 反馈的互相关。互相关估计包含驱动和反馈通道，低相关或最优值贴搜索边界时不可当作精确执行延迟。网格和匀速分析都可能返回“样本不足”，特别是 ±0.10 rad 的短程高速度往返；据实测速度、温度、重叠角度决定是否补测。

单腿回放使用 CAN 反馈力矩和各轴反馈时间、CAD 刚体惯量与闭链装配，步长 1 ms。用一组参数拟合多份无簧试次，分别报告独立验证的角度/速度 RMSE，导出轨迹叠图数据。搜索的是相对 CAD 的**附加轴端惯量**。运动库仑摩擦与阻尼由三角往返先估，未测的被动铰损耗仍是模型假设。候选惯量需有硬件依据；如果优解贴边或换姿态立即变差，不能直接写入训练配置。将 CAN 反馈当作力矩输入时无需再给它添加指令下发延迟。

后续有带簧数据后，用 `--spring installed` 独立解码、汇总，生成 `bench_installed.xml`。将带簧汇总作为 `bench_static.py --installed ... --installed-model ...` 的参数，可在无簧报告基础上导出配对后的轴向力、行程和残差。该模型保留弹簧实体、质量和滑动自由度，推力曲线尚待实测；仅加载 XML 不会产生实际气弹簧推力。
