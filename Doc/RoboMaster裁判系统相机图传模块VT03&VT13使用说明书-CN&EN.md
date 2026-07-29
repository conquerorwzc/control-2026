
Video Transmission Module 图传模块 (VT03&VT13) 

User Guide 使用说明 v1.0 2025.04 

**==> picture [53 x 141] intentionally omitted <==**

## **Contents** 

|**tents**||
|---|---|
|**Disclaimer**|**2**|
|**Introduction**|**2**|
|**In the Box**|**2**|
|**Overview**|**3**|
|VTM Transmitter|3|
|VTM Receiver|3|
|**Using the Product**|**4**|
|Downloading Software|4|
|Installation|4|
|Linking|5|
|Remote Control Data|6|
|VTM Sync Mode|9|
|Firmware Update|9|
|Other Precautions|10|
|**LEDs**|**10**|
|Transmitter|10|
|Receiver|11|
|**Specifications**|**11**|



## **目录** 

|免责声明|**12**|
|---|---|
|简介|**12**|
|物品清单|**12**|
|部件名称|**13**|
|发送端|13|
|接收端|13|
|产品使用|**14**|
|下载软件|14|
|安装|14|
|对频连接|15|
|遥控数据|16|
|同步运行模式|18|
|固件升级|18|
|其它注意事项|19|
|指示灯|**19**|
|发送端|19|
|接收端|19|
|参数|**20**|



1 

## **Disclaimer** 

By using this product, you signify that you have read, understand, and accept the terms and conditions of this guideline and all instructions at https://www.robomaster.com. THE PRODUCT AND ALL MATERIALS AND CONTENT AVAILABLE THROUGH THE PRODUCT ARE PROVIDED "AS IS" AND ON AN "AS AVAILABLE" BASIS WITHOUT WARRANTY OR CONDITION OF ANY KIND. 

## **Introduction** 

RoboMaster Referee System Video Transmission Module (including the transmitter and the receiver, hereinafter referred to as the "VTM") is an image capture and wireless transmission system featuring high resolution, high frame rate, and low latency. Multiple VTMs can work simultaneously. 

As a component of the RoboMaster University Series Referee System (hereinafter referred to as the "Referee System"), the VTM provides a first-person view of robots. The VTM cannot be used independently. 

## **In the Box** 

## **For the transmitter's package:** 

VTM Transmitter 

**==> picture [185 x 8] intentionally omitted <==**

**----- Start of picture text -----**<br>
× 1 Aviation Connector Cable × 1<br>**----- End of picture text -----**<br>


**==> picture [36 x 94] intentionally omitted <==**

## **For the receiver's package:** 

VTM Receiver 

× 1 

2 

## **Overview** 

## ~~VTM Transmitter~~ 

**==> picture [326 x 152] intentionally omitted <==**

**----- Start of picture text -----**<br>
3<br>2 4<br>1 5<br>6 TX<br>GND RX<br>7<br>**----- End of picture text -----**<br>


1. Status LED 

2. Air Inlet 

3. Lens 

4. Air Outlet 

5. USB-C Port 

6. UART Port 

7. Aviation Connector Cable Port 

## ~~VTM Receiver~~ 

**==> picture [91 x 114] intentionally omitted <==**

**----- Start of picture text -----**<br>
8<br>6<br>5<br>2<br>3 7<br>1<br>4<br>9<br>10<br>**----- End of picture text -----**<br>


1. Power Button 

2. Mode Switch 

3. Pause Button 

4. Battery Level LEDs 

5. Control Sticks 

6. Customizable Button (Left) 

   - 11 12 

   - 7. Customizable Button (Right) 8. Antennas 

   9. USB-C Port 

   10. Control Stick Storage Slots 

   11. Dial 

   12. Trigger 

- Certain button functions can be customized. For details, see the Remote Control Data section. 

3 

## **Using the Product** 

## ~~Downloading Software~~ 

Visit https://www.robomaster.com/en-US/products/components/referee to download the required software. 

**==> picture [336 x 80] intentionally omitted <==**

**----- Start of picture text -----**<br>
Software Description<br>Used to update the transmitter firmware, set the running mode,<br>RoboMaster Tool 2<br>and calibrate receiver channels<br>RoboMaster Client Used to view the transmitted images<br>USB Driver* Used to view the transmitted images<br>DJI Assistant 2  Used to upgrade the core version of the transmitter and<br>(RoboMaster Series) update the receiver firmware<br>**----- End of picture text -----**<br>


- The driver will be automatically installed when you install the DJI Assistant 2 (RoboMaster Series) software. If the software is already installed on your computer, installing the driver is unnecessary. 

## ~~Installation~~ 

Refer to the following figure to install the transmitter based on your needs. Use M3 screws to mount the transmitter to an appropriate position. 

- DO NOT block the air inlet ( ① ) and air outlets ( ② ) of the transmitter. 

- Make sure the specified spherical area around the antennas of the transmitter ( ③ ) is free of metal obstructions. 

**==> picture [319 x 232] intentionally omitted <==**

**----- Start of picture text -----**<br>
60.42 Unit: mm<br>54.5<br>32.00<br>3 48 3<br>Through Holes 4- 3.20 Threaded Holes 4-M3 4<br>28.8<br>1<br>8.5 37.55<br>8.5<br>2<br>15 9.2 9.2<br>SR70<br>SR70<br>14.3 25.50<br>14.3<br>45.4 50 35.00 78.5<br>32 32<br>26 26<br>14 9<br>2<br>**----- End of picture text -----**<br>


4 

The receiver does not require installation. The following size information is for reference. 

**==> picture [29 x 5] intentionally omitted <==**

**----- Start of picture text -----**<br>
Unit: mm<br>**----- End of picture text -----**<br>


**==> picture [188 x 83] intentionally omitted <==**

**----- Start of picture text -----**<br>
150 65<br>103<br>**----- End of picture text -----**<br>


**==> picture [5 x 9] intentionally omitted <==**

**----- Start of picture text -----**<br>
203<br>**----- End of picture text -----**<br>


## ~~Linking~~ 

## **China and Other Non-Japan Regions** 

1. Ensure the Power Management Module is connected to the Main Controller Module in the Referee System. 

2. **Power on the transmitter:** Connect the transmitter to the Power Management Module using the Aviation Connector Cable included in the package. Once the transmitter is powered on, the status LED flashes red once per second. 

3. **Power on the receiver:** Press the power button once, then press and hold for 2 seconds. 

4. **Start linking:** 

   - **For the transmitter:** Tap **Module Setup > VTM Linking** on the Main Controller Module. During the linking process, the status LED flashes blue once per second. 

   - **For the receiver:** Press and hold the power button until you hear a "beep" sound after about 2 seconds, briefly release the button, and then press and hold the button again for about 2 seconds. During the linking process, the receiver continues to beep. 

   - Once linked, the status LED of the transmitter flashes green once per second, and the receiver beeps twice. The linking operation is required only once. 

5. Connect the USB-C port of the receiver to a computer and open the RoboMaster Client software. You can view the transmitted images. 

## **Japan** 

1. Ensure the Power Management Module is connected to the Main Controller Module in the Referee System. 

2. **Power on the transmitter:** Connect the transmitter to the Power Management Module using the Aviation Connector Cable included in the package. Once the transmitter is powered on, the status LED turns solid orange. 

5 

3. **Power on the receiver:** Press the power button once, then press and hold for 2 seconds. 

4. Correctly set the robot ID on the Main Controller Module. 

5. Connect the USB-C port of the receiver to a computer. Open the RoboMaster Client software, select the corresponding robot ID, and click **Login** to view the transmitted images. 

   - The receiver will remain linked with the transmitter until the receiver is restarted. After a restart, you need to repeat the linking operation by following step 3 to 5. 

## ~~Remote Control Data~~ 

Once linked with the transmitter, the receiver can function as a remote controller and send remote control data. The receiver can also send certain keyboard and mouse data (see the Data Frame Structure Table). To do so, connect the receiver to a computer, open the RoboMaster Client software, and log in to the corresponding robot. You can customize the functions of the control sticks and buttons (excluding the power button) on the receiver as well as the keys on your computer’s keyboard and mouse. 

Once linked with the receiver, the transmitter outputs a 21-byte data frame every 14 ms through the UART port. The communication parameters are listed in the following table. 

**==> picture [335 x 64] intentionally omitted <==**

**----- Start of picture text -----**<br>
Serial Port Parameters Value<br>Baud Rate 921600<br>Data Bits 8<br>Stop Bit 1<br>Parity Bit None<br>Flow Control None<br>**----- End of picture text -----**<br>


## Data Frame Structure Table: 

**==> picture [336 x 32] intentionally omitted <==**

**----- Start of picture text -----**<br>
Length<br>Domain Offset Sign Bit Value Description<br>(Bits)<br>Header 1 0 8 None 0xA9 Fixed value<br>**----- End of picture text -----**<br>


|**Domain**|**Offset**|**Length**<br>**(Bits)**|**Sign Bit **|**Value**|**Description**|
|---|---|---|---|---|---|
|Header 1|0|8|None|0xA9|Fixed value|
|Header 2|8|8|None|0x53|Fixed value|
|Channel 0|16|11|None|Minimum: 364<br>Central: 1024<br>Maximum: 1684|The horizontal position of the<br>receiver's right control stick|
|Channel 1|27|11|None|Minimum: 364<br>Central: 1024<br>Maximum: 1684|The vertical position of the<br>receiver's right control stick|
|Channel 2|38|11|None|Minimum: 364<br>Central: 1024<br>Maximum: 1684|The vertical position of the<br>receiver's left control stick|



6 

|Channel 3|49|11|None|Minimum: 364<br>Central: 1024<br>Maximum: 1684|The horizontal position of the<br>receiver's left control stick|
|---|---|---|---|---|---|
|Mode switch|60|2|None|Minimum: 0<br>Maximum: 2|The position of the receiver's<br>mode switch:<br>C: 0<br>N: 1<br>S: 2|
|Pause button|62|1|None|Minimum: 0<br>Maximum: 1|Whether the receiver's pause<br>button is pressed:<br>Not pressed: 0<br>Pressed: 1|
|Customizable<br>button (left)|<br>63|1|None|Minimum: 0<br>Maximum: 1|Whether the receiver's<br>customizable button (left) is<br>pressed:<br>Not pressed: 0<br>Pressed: 1|
|Customizable<br>button (right)|<br>64|1|None|Minimum: 0<br>Maximum: 1|Whether the receiver's<br>customizable button (right) is<br>pressed:<br>Not pressed: 0<br>Pressed: 1|
|Dial|65|11|None|Minimum: 364<br>Central: 1024<br>Maximum: 1684|The position of the receiver's<br>dial|
|Trigger|76|1|None|Minimum: 0<br>Maximum: 1|Whether the receiver's trigger<br>is pressed:<br>Not pressed: 0<br>Pressed: 1|
|X-axis mouse<br>movement|80|16|Present|Minimum: -32768<br>Static: 0<br>Maximum: 32767|The left or right movement<br>speed of the mouse<br>(A negative value indicates<br>left movement)|
|Y-axis mouse<br>movement|96|16|Present|Minimum: -32768<br>Static: 0<br>Maximum: 32767|The forward or backward<br>movement speed of the<br>mouse<br>(A negative value indicates<br>backward movement)|
|Z-axis mouse<br>movement|112|16|Present|Minimum: -32768<br>Static: 0<br>Maximum: 32767|The speed of the mouse's<br>scroll wheel<br>(A negative value indicates<br>backward scrolling)|



7 

|Left mouse<br>button|128|2|None|Minimum: 0<br>Maximum: 1|Whether the left mouse<br>button is pressed:<br>Not pressed: 0<br>Pressed: 1|
|---|---|---|---|---|---|
|Right mouse<br>button|130|2|None|Minimum: 0<br>Maximum: 1|Whether the right mouse<br>button is pressed:<br>Not pressed: 0<br>Pressed: 1|
|Middle<br>mouse<br>button|132|2|None|Minimum: 0<br>Maximum: 1|Whether the middle mouse<br>button is pressed:<br>Not pressed: 0<br>Pressed: 1|
|Keyboard|136|16|None|Minimum: 0<br>Maximum: 65535|Keyboard key status: Each<br>bit represents a key, where<br>0 indicates the key is not<br>pressed and 1 indicates it is<br>pressed.<br>bit0: W key<br>bit1: S key<br>bit2: A key<br>bit3: D key<br>bit4: Shift key<br>bit5: Ctrl key<br>bit6: Q key<br>bit7: E key<br>bit8: R key<br>bit9: F key<br>bit10: G key<br>bit11: Z key<br>bit12: X key<br>bit13: C key<br>bit14: V key<br>bit15: B key|
|CRC|152|16|None|N/A|This Cyclic Redundancy Check<br>(CRC) uses the standard<br>CRC-16/CCITT-FALSE<br>polynomial P(x)=x<br>16+x<br>12+x<br>5+1<br>(corresponding to 0x1021),<br>with an initial value of 0xFFFF.<br>Input and output data are<br>not inverted, and no XOR<br>operation is applied.|



8 

Sample code for data frame and CRC: 

Visit https://www.robomaster.com/en-US/products/components/referee to download. 

## ~~VTM Sync Mode~~ 

- This feature is not supported in Japan. 

When using multiple VTMs at the same time, you can enable sync mode to enhance video transmission performance and reduce mutual interference. 

1. **Set up a sync anchor:** Connect a VTM transmitter to a computer using the Main Controller Module. On the computer, open the RoboMaster Tool 2 software, click **Sync Anchor Setup** , and configure the transmitter as a sync anchor. 

2. **Enable sync mode:** Choose **Debug Settings > VTM Sync Mode** on the Main Controller Module. 

## ~~Firmware Update~~ 

## **Transmitter Core Version/Receiver Firmware** 

Use the DJI Assistant 2 (RoboMaster Series) software to update the transmitter core version and the receiver firmware separately. 

1. Power on the device and connect it to a computer using the USB-C Port. 

2. Launch DJI Assistant 2 (RoboMaster Series), log in with your DJI account, and enter the main interface. 

3. Select the device and click **Firmware Update** on the left side of the screen. 

4. Select and confirm the firmware version to update to. 

5. Wait for the firmware to download. The firmware update will start automatically. 

6. Wait for the update to complete. 

   - Make sure to follow all the steps; otherwise the update may fail. 

   - Make sure the computer is connected to the internet during the update. 

   - DO NOT unplug the USB-C cable during an update. 

   - Before performing an update for the receiver, make sure the receiver is sufficiently charged. 

## **Transmitter Firmware** 

Use RoboMaster Tool 2 to update the transmitter firmware. The operation is the same as the other modules of the Referee System. For details, refer to the User Manual of the Referee System. 

9 

## ~~Other Precautions~~ 

1. Make sure each port is properly connected according to the instructions to prevent malfunctions or damage. 

2. When the VTM is in use, keep it away from wireless devices operating in the 5 GHz frequency band to ensure video transmission quality. 

3. A maximum of eight VTMs can be used simultaneously in the same area. Exceeding this limit may lead to poor image quality, image freezing, or even disconnection. It is recommended to enable sync mode when using multiple VTMs simultaneously. 

## **LEDs** 

## ~~Transmitter~~ 

**==> picture [335 x 283] intentionally omitted <==**

**----- Start of picture text -----**<br>
Activation and power status<br>Purple light Solid Not activated<br>Purple light Flashing once per second Not powered on<br>Purple light Not connected to the Referee<br>Flashing 5 times per second<br>System's Main Controller Module<br>Linking status (China and other non-Japan regions)<br>Red light Flashing once per second Not linked with the receiver<br>Blue light Flashing once per second Linking with the receiver<br>Green light Flashing once per second Linked with the receiver<br>Yellow light Operating in sync mode but no<br>Flashing once per second<br>sync anchor is found<br>Linking status (Japan)<br>Orange light Flashing once per second Not linked with the receiver<br>Orange light Flashing once per second Linking with the receiver<br>Cyan light Solid Linked with the receiver<br>Cyan light Operating in sync mode but no<br>Flashing once per second<br>sync anchor is found<br>Operating status as a sync anchor (China and other non-Japan regions)<br>Cyan light and red<br>Flashing alternately Error<br>light<br>Cyan light Flashing 5 times per second Normal<br>**----- End of picture text -----**<br>


10 

## ~~Receiver~~ 

**==> picture [335 x 64] intentionally omitted <==**

**----- Start of picture text -----**<br>
Blinking Pattern Battery Level<br>76-100%<br>51-75%<br>26-50%<br>0-25%<br>**----- End of picture text -----**<br>


During the linking process, the four battery level LEDs blink in sequence. After successful linking, the LEDs stay lit. 

## **Specifications** 

**==> picture [335 x 365] intentionally omitted <==**

**----- Start of picture text -----**<br>
VTM<br>Operating Temperature -10° to 40° C (14° to 104° F)<br>Operating Frequency  5150-5250 MHz (indoor use only)<br>(China and other non-Japan regions) 5732-5829 MHz<br>Operating Frequency (Japan) 5150-5250 MHz, 5650-5755 MHz<br>Video Resolution Maximum: 1920×1200<br>Video Frame Rate Maximum: 60 Hz<br>End-to-End Latency <90 ms (laboratory environment)<br>Max Transmission Range 100 m (without any obstruction)<br>Transmitter<br>Weight Approx. 121 g<br>Input Voltage 12 V<br>Operating Current 665 mA<br>Image Sensor 1/1.3-inch CMOS sensor<br>Lens Field of View (FOV): approx. 139°<br>Equivalent focal length: 12 mm<br>Aperture: f/2.8<br>Focus: 0.6 m to ∞<br>ISO Range 100-25600 (Auto)<br>Receiver<br>Weight Approx. 375 g<br>Max Operating Time 6 hours<br>Charging Temperature 5° to 40° C (41° to 104° F)<br>Charging Time 2.5 hours<br>Charging Method 5 V/2 A charger (recommended)<br>Battery Capacity 18.72 Wh (3.6 V, 2600 mAh×2)<br>Battery Type 18650 Li-ion<br>**----- End of picture text -----**<br>


11 

## **免责声明** 

使用本产品前，请您仔细阅读本文、访问 https://www.robomaster.com 阅读本产品相关的所 有指引。使用本产品视为您已经阅读并接受本文与本产品相关的全部条款。 

## **简介** 

RoboMaster 裁判系统相机图传模块（包括发送端和接收端，以下简称为“图传模块”）是一 套实时高清图像采集和无线传输系统，具有高清晰度、高帧率、低延迟的特点，并且支持多套 图传模块同时工作。 

图传模块是 RoboMaster 高校系列赛裁判系统的组成部分，可以为用户提供机器人的第一人称 视角。图传模块不可独立使用。 

## **物品清单** 

## 发送端物品清单： 

相机图传模块发送端 

> × 1 航空转接线 × 1 

**==> picture [36 x 93] intentionally omitted <==**

## 接收端物品清单： 

相机图传模块接收端 

× 1 

12 

## **部件名称** 

## ~~发送端~~ 

**==> picture [326 x 187] intentionally omitted <==**

**----- Start of picture text -----**<br>
3<br>2 4<br>1 5<br>6 TX<br>GND RX<br>7<br>1.  状态指示灯 4.  出风口 7.  航空转接线接口<br>2.  进风口 5. USB-C  接口<br>**----- End of picture text -----**<br>


**==> picture [26 x 9] intentionally omitted <==**

**----- Start of picture text -----**<br>
3.  镜头<br>**----- End of picture text -----**<br>


**==> picture [48 x 9] intentionally omitted <==**

**----- Start of picture text -----**<br>
6. UART  接口<br>**----- End of picture text -----**<br>


## ~~接收端~~ 

**==> picture [99 x 136] intentionally omitted <==**

**----- Start of picture text -----**<br>
8<br>6<br>5<br>2<br>3 7<br>1<br>4<br>9<br>10<br>1.  电源按键<br>**----- End of picture text -----**<br>


2. 挡位切换开关 

3. 暂停按键 

4. 电量指示灯 

5. 摇杆 

6. 自定义按键（左） 

   - 11 12 

   - 7. 自定义按键（右） 8. 天线 9. USB-C 接口 10. 摇杆收纳槽 11. 拨轮 12. 扳机键 

- 部分按键功能支持自定义。详见“遥控数据”章节。 

13 

## **产品使用** 

## ~~下载软件~~ 

**==> picture [336 x 81] intentionally omitted <==**

**----- Start of picture text -----**<br>
请前往 https://www.robomaster.com/zh-CN/products/components/referee  下载所需软件。<br>软件 描述<br>RoboMaster Tool 2 用于发送端固件升级、运行模式设置、接收端通道校准等<br>RoboMaster Client 用于显示图传画面<br>USB  驱动 * 用于显示图传画面<br>DJI Assistant 2<br>用于发送端核心版本升级、接收端固件升级等<br>（ RoboMaster Series ）<br>**----- End of picture text -----**<br>


- 安装DJI Assistant 2（RoboMaster Series）软件时会自动安装该驱动。如果电脑已安装该软件，无需重 复安装该驱动。 

## ~~安装~~ 

参考如下发送端结构尺寸图，按需选择孔位进行安装。安装时，请使用 M3 螺丝固定发送端至 适当位置。 

- 不能遮挡发送端的进风口（①）与出风口（②）。 

- 发送端的天线（③）周围不能有任何金属遮挡。 

**==> picture [32 x 7] intentionally omitted <==**

**----- Start of picture text -----**<br>
单位： mm<br>**----- End of picture text -----**<br>


**==> picture [305 x 228] intentionally omitted <==**

**----- Start of picture text -----**<br>
60.42<br>54.5<br>32.00<br>3 48 3<br>通孔 4- 3.20 螺纹孔 4-M3 4<br>28.8<br>1<br>8.5 37.55<br>8.5<br>2<br>15 9.2 9.2<br>SR70<br>SR70<br>14.3 25.50<br>14.3<br>45.4 50 35.00 78.5<br>32 32<br>26 26<br>14 9<br>2<br>**----- End of picture text -----**<br>


14 

## 接收端无需安装，以下尺寸信息作为参考。 

## 单位： mm 

**==> picture [188 x 84] intentionally omitted <==**

**----- Start of picture text -----**<br>
150 65<br>103<br>**----- End of picture text -----**<br>


**==> picture [5 x 9] intentionally omitted <==**

**----- Start of picture text -----**<br>
203<br>**----- End of picture text -----**<br>


## ~~对频连接~~ 

## 中国及其它非日本地区 

1. 确保裁判系统的电源管理模块已连接至主控模块。 

2. 启动发送端：使用包装内附带的航空转接线将发送端连接到裁判系统的电源管理模块。成功 启动后，发送端状态指示灯变为红灯闪烁（每 1 秒闪 1 次）。 

3. 启动接收端：短按 1 次电源按键，再长按 2 秒。 

## 4. 触发对频： 

   - 发送端：在裁判系统主控模块上点击模块设置 **>** 图传对频。触发对频后，发送端状态指示灯 变为蓝灯闪烁（每 1 秒闪 1 次）。 

   - 接收端：先长按电源键直至听到“嘀”的一声（约 2 秒），短暂松开后再次长按电源键（约 2 秒）。触发对频后，接受端会持续发出“嘀…嘀…嘀”的提示音。 

   - 当发送端状态指示灯变为绿灯闪烁（每 1 秒闪 1 次）、接收端发出“嘀嘀”两声后静音时， 代表对频成功。后续使用无需再次对频。 

5. 连接接收端的 USB-C 接口至电脑，打开 RoboMaster Client 软件，即可查看图像画面。 

## 日本地区 

1. 确保裁判系统的电源管理模块已连接至主控模块。 

2. 启动发送端：使用包装内附带的航空转接线将发送端连接到裁判系统的电源管理模块。成功 启动后，发送端状态指示灯变为橙灯常亮。 

3. 启动接收端：短按 1 次电源按键，再长按 2 秒。 

4. 在裁判系统主控模块上正确设置机器人 ID 。 

5. 连接接收端的 USB-C 接口至电脑。打开 RoboMaster Client 软件，选择对应的机器人 ID ， 点击登录，即可查看图像画面。 

   - 接收端在断电重启前会保持与发送端的连接状态，断电重启后需要与发送端重新对频连 接，即重复步骤 3 至步骤 5 的操作。 

15 

## ~~遥控数据~~ 

与发送端对频连接后，接收端可作为遥控器发送遥控数据。将接收端连接至电脑，打开 RoboMaster Client 软件并登录对应机器人后，开放的键鼠数据（详见下方数据帧结构表）也 会通过接收端发送。遥控器按键（电源键除外）和摇杆、电脑键鼠数据的功能均支持自定义。 与接收端对频连接后，发送端每间隔 14ms 通过 UART 接口输出一帧 21 字节的数据，通信参数 如下表所示。 

**==> picture [335 x 64] intentionally omitted <==**

**----- Start of picture text -----**<br>
串口参数 数值<br>波特率 921600<br>数据位 8<br>停止位 1<br>校验位 无<br>流控 无<br>**----- End of picture text -----**<br>


## 数据帧结构表： 

**==> picture [336 x 21] intentionally omitted <==**

**----- Start of picture text -----**<br>
域 偏移长度（位）符号位取值 描述<br>帧头 1 0 8 无 0xA9 固定值<br>**----- End of picture text -----**<br>


|域|偏移|长度（位）|符号位|取值|描述|
|---|---|---|---|---|---|
|帧头1|0|8|无|0xA9|固定值|
|帧头2|8|8|无|0x53|固定值|
|通道0|16|11|无|最小值：364<br>中间值：1024<br>最大值：1684|接收端右摇杆水平方向的位置|
|通道1|27|11|无|最小值：364<br>中间值：1024<br>最大值：1684|接收端右摇杆竖直方向的位置|
|通道2|38|11|无|最小值：364<br>中间值：1024<br>最大值：1684|接收端左摇杆竖直方向的位置|
|通道3|49|11|无|最小值：364<br>中间值：1024<br>最大值：1684|接收端左摇杆水平方向的位置|
|挡位切换开关|60|2|无|最小值：0<br>最大值：2|接收端挡位切换开关位置：<br>C：0<br>N：1<br>S：2|
|暂停按键|62|1|无|最小值：0<br>最大值：1|接收端暂停按键是否按下：<br>未按下：0<br>按下：1|
|自定义按键<br>（左）|63|1|无|最小值：0<br>最大值：1|接收端自定义按键（左）是否按下：<br>未按下：0<br>按下：1|
|自定义按键<br>（右）|64|1|无|最小值：0<br>最大值：1|接收端自定义按键（右）是否按下：<br>未按下：0<br>按下：1|



16 

|拨轮|65|11|无|最小值：364<br>中间值：1024<br>最大值：1684|接收端拨轮位置|
|---|---|---|---|---|---|
|扳机键|76|1|无|最小值：0<br>最大值：1|接收端扳机键是否按下：<br>未按下：0<br>按下：1|
|鼠标X轴|80|16|有|最小值：-32768<br>静止值：0<br>最大值：32767|鼠标左右移动的速度<br>（负值表示向左移动）|
|鼠标Y轴|96|16|有|最小值：-32768<br>静止值：0<br>最大值：32767|鼠标前后移动的速度<br>（负值表示向后移动）|
|鼠标Z轴|112|16|有|最小值：-32768<br>静止值：0<br>最大值：32767|鼠标滚轮的滚动速度<br>（负值表示向后滚动）|
|鼠标左键|128|2|无|最小值：0<br>最大值：1|鼠标左键是否按下：<br>未按下：0<br>按下：1|
|鼠标右键|130|2|无|最小值：0<br>最大值：1|鼠标右键是否按下：<br>未按下：0<br>按下：1|
|鼠标中键|132|2|无|最小值：0<br>最大值：1|鼠标中键是否按下：<br>未按下：0<br>按下：1|
|键盘按键|136|16|无|最小值：0<br>最大值：65535|键盘按键信息，每个bit对应一个<br>按键，0为未按下，1为按下。<br>bit0：W键<br>bit1：S键<br>bit2：A键<br>bit3：D键<br>bit4：Shift键<br>bit5：Ctrl键<br>bit6：Q键<br>bit7：E键<br>bit8：R键<br>bit9：F键<br>bit10：G键<br>bit11：Z键<br>bit12：X键<br>bit13：C键<br>bit14：V键<br>bit15：B 键|



17 

|CRC校验|152|16|无|N/A|循环冗余校验（Cyclic<br>Redundancy Check, CRC）。使<br>用标准CRC-16/CCITT-FALSE多<br>项式P(x)=x<br>16+x<br>12+x<br>5+1（对应<br>0x1021），初始值为0xFFFF，无<br>输入输出反转，无异或。|
|---|---|---|---|---|---|



## 数据帧及校验示例代码： 

前往 https://www.robomaster.com/zh-CN/products/components/referee 下载。 

## ~~同步运行模式~~ 

- 日本地区不支持该功能。 

同时使用多套图传模块时，可以开启同步运行模式以提升图传性能、减小图传间的干扰。 

1. 准备同步桩：将一台发送端通过主控模块连接至电脑，打开 RoboMaster Tool 2 软件，选择 同步桩设置，将发送端配置为同步桩。 

2. 开启同步运行模式：在裁判系统主控模块上勾选调试设置 **>** 图传同步运行模式。 

## ~~固件升级~~ 

## 发送端核心版本 **/** 接收端固件 

使用 DJI Assistant 2 （ RoboMaster Series ）软件分别升级发送端与接收端。 

1. 开启设备，将设备通过 USB-C 接口连接至电脑。 

2. 启动 DJI Assistant 2 （ RoboMaster Series ）软件，用 DJI 账号登陆并进入主界面。 

3. 点击设备图标，然后点击左边的固件升级选项。 

4. 选择并确认需要升级的固件版本。 

5. 软件将自行下载并升级固件。 

6. 等待升级完成即可。 

   - 确保按步骤升级固件，否则可能导致升级失败。 

   - 确保整个升级过程中电脑能够访问互联网。 

   - 升级过程中请勿插拔 USB 数据线。 

   - 升级接收端固件时，确保接收端电量充足。 

## 发送端固件 

- 发送端固件通过 RoboMaster Tool 2 升级。升级方式与裁判系统其它模块相同，详见 《 RoboMaster 裁判系统用户手册》。 

18 

## ~~其它注意事项~~ 

1. 相机图传模块在使用过程中，请确保各接口按照说明正确连接，以免模块工作异常，甚至导 致模块损坏。 

2. 相机图传模块在使用过程中，请注意避开 5 GHz 频段无线设备的干扰，以免影响图像传输质量。 

3. 同一环境下最多可同时使用 8 套相机图传模块，否则可能导致图像质量下降，甚至发生卡顿 或断连。多套图传模块同时工作时，建议使用同步运行模式。 

## **指示灯** 

**==> picture [351 x 258] intentionally omitted <==**

**----- Start of picture text -----**<br>
发送端<br>激活与启动状态<br>紫灯 常亮 未激活<br>紫灯 每 1  秒闪 1  次 未启动<br>紫灯 每 1  秒闪 5  次 未连接裁判系统主控模块<br>连接状态（中国及其它非日本地区）<br>红灯 每 1  秒闪 1  次 未连接接收端<br>蓝灯 每 1  秒闪 1  次 与接收端对频中<br>绿灯 每 1  秒闪 1  次 已连接接收端<br>黄灯 每 1  秒闪 1  次 已进入同步运行模式但未找到同步桩<br>连接状态（日本地区）<br>橙灯 每 1  秒闪 1  次 未连接接收端<br>橙灯 每 1  秒闪 1  次 与接收端对频中<br>青灯 常亮 已连接接收端<br>青灯 每 1  秒闪 1  次 已进入同步运行模式但未找到同步桩<br>作为同步桩的工作状态（中国及其它非日本地区）<br>青灯与红灯 交替闪烁 状态异常<br>青灯 每 1  秒闪 5  次 同步正常<br>**----- End of picture text -----**<br>


## ~~接收端~~ 

**==> picture [336 x 65] intentionally omitted <==**

**----- Start of picture text -----**<br>
闪灯方式 电量<br>76%-100%<br>51%-75%<br>26%-50%<br>0%-25%<br>**----- End of picture text -----**<br>


对频过程中，接收端的 4 颗电量指示灯循环闪烁；对频成功后，接收端电量指示灯变为常亮。 

19 

## **参数** 

**==> picture [335 x 366] intentionally omitted <==**

**----- Start of picture text -----**<br>
图传模块<br>工作环境温度 -10 ℃至 40 ℃<br>工作频段（中国及其它非日本地区） 5150-5250 MHz （仅限室内使用）<br>5732-5829 MHz<br>工作频段（日本地区） 5150-5250 MHz ， 5650-5755 MHz<br>传输图像分辨率 最大 1920 × 1080<br>传输图像帧率 最大 60 Hz<br>端到端延时 < 90 ms （实验室环境）<br>最远传输距离 100 m （无遮挡时）<br>发送端<br>重量 约 121 g<br>供电电压 12 V<br>工作电流 665 mA<br>影像传感器 1/1.3  英寸影像传感器<br>镜头 视角范围（ FOV ）：约 139 °<br>等效焦距： 12  毫米<br>光圈： f/2.8<br>对焦点： 0.6  米至无穷远<br>ISO  范围 100  至  25600 （自动）<br>接收端<br>重量 约 375 g<br>最长续航时间 6  小时<br>充电环境温度 5 ℃至 40 ℃<br>充电时间 2.5  小时<br>充电方式 建议使用 5 V/2 A  的充电器<br>电池容量 18.72 Wh (3.6 V, 2600 mAh × 2)<br>电池类型 18650  锂离子电池<br>**----- End of picture text -----**<br>


20 

如果您对说明书有任何疑问或建议，请通过 DocSupport@dji.com 联系我们。 If you have any questions about this document, please contact DJI by sending a message to DocSupport@dji.com. 

RoboMaster and are trademarks of DJI. Copyright © 2025 DJI All Rights Reserved. RoboMaster 和 是大疆创新的商标。 Copyright © 2025 大疆创新 版权所有 

