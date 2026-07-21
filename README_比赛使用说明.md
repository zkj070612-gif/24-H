# YB-CVT01 八路 UART 自动行驶小车（H 题）

## 工程与硬件

- MCU：MSPM0G3507SPMR，Keil MDK-ARM 6.22。
- SDK：`C:\ti\mspm0_sdk_2_10_00_04`。
- 八路循迹模块使用 UART0，115200 bit/s：模块 TX 接 RX0（PA11），模块 RX 接 TX0（PA10）。
- MPU6050 使用板载 IMU 插座：SDA=PA0，SCL=PA1。
- 左电机接 Motor1，右电机接 Motor2。
- UART0 已专用于循迹模块，连接模块时不要使用 USB 串口调试，也不要向 UART0 输出 printf 文本。

## 上电操作

1. 小车静止上电，等待 MPU6050 DMP 初始化完成。
2. OLED 进入模式选择：KEY1 在 1~4 间循环，KEY2 确认。
3. 程序保证八路模块上电稳定至少 20 秒，然后发送 `$0,0,1#` 开启数字量连续输出。
4. 收到完整 `$D,x1:...,x8:...#` 帧后才允许起步；UART 超过 500 ms 无有效帧会自动刹车并亮红灯。

模式含义：

- 1：A→B 停车。
- 2：A→B→C→D→A，完成一圈停车。
- 3：A→C→B→D→A，完成一圈停车。
- 4：按模式 3 路径连续完成四圈停车。

程序只使用前进差速和刹车，不执行倒车或原地反转。

## 赛前必须校准

八路模块应先在实际白色哑光底板和 1.8 cm 黑线上完成黑白校准。OLED 第三行实时显示 x1~x8；正常情况下白底全为 1，压到黑线的探头为 0。

对角线修正角集中在 `BSP/control/control.c`：

- `s_first_diagonal_deg[]`：A→C 的每圈角度修正。
- `s_second_diagonal_deg[]`：B→D 的每圈角度修正。

如果第一次 A→C 转向方向相反，将两组角度的正负号同时反转。循迹 PID 和速度在 `APP/app_irtracking_eight.c`；直线速度和航向 PID 在 `BSP/control/control.c`。

## Keil 与 UniFlash

Keil 打开 `keil\YB_CVT01_H8.uvprojx`，执行 Rebuild。输出文件为 `OBJ\YB_CVT01_H8.hex`。

UniFlash 选择 MSPM0G3507 对应调试器，连接后在 Program 页面选择该 Intel HEX，执行 Load Image。烧录前应抬起驱动轮；烧录完成、断开调试连接后再把小车放到 A 点上电测试。
