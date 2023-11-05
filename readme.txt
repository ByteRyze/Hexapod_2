这是六足机器人项目：
注意事项：
1. 在main函数第一行加入SCB->VTOR = 0x90000000; /* 设置中断向量表地址 */
2. 注释掉MPU_Config
3. 不要修改和qspi有关的任何参数
4. HCLK3必须维持在120MHz

论文还在写，算法描述已经写完了。欢迎来b站催更

核心代码路径MDK-ARM/USER,机器人运动解算在gait_prg.cpp和gait_prg.h中
