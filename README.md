# 24-H 自动行驶小车

基于 MSPM0G3507（Yahboom YB-CVT01-V1.0 小车）的江苏省电赛自动行驶小车 H 题工程。

## 打开与编译

1. 安装 TI MSPM0 SDK `2.10.00.04` 和 Keil MDK。
2. 用 Keil 打开 `keil/YB_CVT01_H8.uvprojx`。
3. 编译后，HEX 文件将在 `OBJ/YB_CVT01_H8.hex` 生成。
4. 使用 UniFlash 将 HEX 烧录至 MSPM0G3507。

本工程的 Keil 工程引用本机的 MSPM0 SDK；若 SDK 路径不同，请在 Keil 的 Include Paths 与 Linker 设置中改为你的 SDK 位置。

## 仓库内容

- `APP/`：任务逻辑与控制算法
- `BSP/`：板级外设驱动
- `keil/`：Keil 工程、启动文件与散装加载文件
- `empty.syscfg`、`ti_msp_dl_config.*`：TI SysConfig 生成配置

编译产物和个人 Keil 配置文件由 `.gitignore` 排除，不会提交到仓库。
