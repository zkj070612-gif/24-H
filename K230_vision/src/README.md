# src

这里存放本项目实际部署到 K230 的 CanMV MicroPython 源码。

## 钢珠识别

`main.py` 是当前钢珠识别入口，使用出厂固件已有的 `cv_lite` 霍夫圆检测，不需要 `.kmodel`。

程序会：

1. 从 640×480 相机画面搜索指定半径范围内的圆。
2. 使用钢珠低饱和度、高光、阴影和圆周边缘完整度过滤普通圆形干扰物。
3. 为每颗钢珠建立独立 ID，连续三帧位置一致后确认。
4. 单颗钢珠短暂漏检时保留其轨迹，不影响其他钢珠。
5. 在屏幕上标出所有稳定钢珠、ID、中心与半径。
6. 通过 UART1 一帧输出全部稳定钢珠。

当前默认开启诊断显示：

- 黄色/绿色圆：通过过滤的钢珠候选或稳定目标。
- 红色圆：被拒绝的霍夫圆候选。
- 红色标注中的 `r/c/s/e`：半径、内部亮度反差、平均饱和度、圆周边缘支持率。
- `LOW_CONTRAST`：内部明暗变化不足。
- `NO_HIGHLIGHT`：没有足够亮的反光。
- `TOO_COLORFUL`：颜色饱和度过高，不像银灰色钢珠。
- `WEAK_EDGE`：圆周只有少部分存在真实边界。
- `OUTSIDE_ROI`：圆没有完整落在检测区域内。

调试时请截一张同时包含真实钢珠和误检圆标注的画面。根据画面中的 `r/c/s/e` 数值即可收紧阈值，而不需要盲目试参数。比赛运行前可将 `SHOW_REJECTED_CIRCLES` 改为 `False`。

首次测试建议在 CanMV IDE 中手动打开并运行本文件，不要立刻覆盖 SD 卡原有的 `main.py`。确认摄像头、屏幕和识别效果正常后，再备份原文件并部署。

需要根据实物优先调整的参数都位于文件开头：

- `MIN_RADIUS`、`MAX_RADIUS`：画面中的钢珠半径范围。
- `DETECTION_ROI`：允许钢珠出现的区域。
- `PICKUP_TARGET_X`、`PICKUP_TARGET_Y`：电磁铁拾取点在画面中的位置，程序优先锁定离它最近的钢珠。
- `HOUGH_ACCUMULATOR_THRESHOLD`：越低越容易检出，也越容易误检。
- `MIN_LUMA_CONTRAST`、`MIN_HIGHLIGHT_LUMA`、`MAX_AVERAGE_SATURATION`：银色钢珠外观过滤。
- `MIN_EDGE_DIFFERENCE`、`MIN_EDGE_SUPPORT_PERCENT`：钢珠圆周边缘过滤。
- `ENABLE_STEEL_APPEARANCE_FILTER`：标定圆检测时可暂时设为 `False`。

2026-07-25 实物截图标定值：

- 检测区域：`(80, 250, 480, 220)`，仅覆盖画面中下部钢珠区域。
- 钢珠候选半径约 `11～21 px`；程序搜索范围设为 `7～26 px`。
- 白色数据线误检半径约 `43～47 px`，已由最大半径和 ROI 排除。
- 多颗钢珠同时出现时，每颗分别维护坐标、半径、命中次数、漏检次数和 ID。
- 黄色 `P编号` 表示仍在确认的临时轨迹；青色 `ID编号` 表示稳定钢珠。
- 绿色 `TARGET ID编号` 是距离拾取点最近且已锁定的操作目标。
- 单颗钢珠短暂漏检时保持约 0.3 秒，连续丢失后才删除该 ID。
- UART 使用 `$BALLS` 帧一次发送全部稳定轨迹，不再只发送一颗。

依赖 SD 卡原厂文件：

- `/sdcard/libs/PipeLine.py`
- `/sdcard/ybUtils/YbUart.py`
- 固件内置 `cv_lite`
