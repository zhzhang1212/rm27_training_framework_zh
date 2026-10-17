# SPI 通信学习笔记

## 1. 先弄懂：全双工与半双工

这里的“工”指的是**数据传输的方向**，不是传输速度。

- **全双工（Full Duplex）**：通信双方可以在**同一时刻**互相发送数据。就像打电话，你说话的同时也能听到对方说话。
- **半双工（Half Duplex）**：双方都能发送和接收，但**同一时刻只能向一个方向传输**。就像对讲机，一方说完，另一方再说。
- **单工（Simplex）**：只能向一个方向传输，例如只能发送、不能接收。

记住：**全双工 ≠ 速度更快；区别在于能否同时双向传输。**

## 2. SPI 是什么？

SPI（Serial Peripheral Interface，串行外设接口）是一种让两颗芯片交换数据的**同步串行通信方式**。

以 C 板为例：STM32 想读取板上 BMI088 测量的数据，就可以通过 SPI 与它通信。

- **串行**：数据一位一位地传，而不是把一个字节的 8 位同时放到 8 根数据线上。
- **同步**：双方根据同一根时钟线提供的节拍传输数据。
- **主机（Master）**：主动发起通信、提供时钟的一方，这里是 STM32。
- **从机（Slave）**：按主机时钟配合通信的设备，这里是 BMI088。

## 3. SPI 的四根线是做什么的？

```text
STM32（主机）                     BMI088（从机）
    SCK  ---------------------->  时钟输入
    MOSI ---------------------->  数据输入
    MISO <----------------------  数据输出
    CS   ---------------------->  片选输入
```

| 信号线 | 最简单的理解 |
| --- | --- |
| SCK | 时钟：STM32 告诉双方什么时候传下一位数据 |
| MOSI | 主机发送、从机接收的数据线 |
| MISO | 从机发送、主机接收的数据线 |
| CS | 片选：告诉某个从机“现在和你通信”，通常低电平有效 |

**为什么 SPI 通常是全双工？** 因为发送和接收各有一根数据线（MOSI、MISO）。每产生一个时钟周期，主机既可以发送一位，也可以接收一位。

## 4. STM32 怎样用 SPI 读取数据？

先不要看代码，假设 STM32 要读取传感器中的某个**寄存器**。寄存器可以简单理解为芯片内部存放配置或测量数据的小位置，每个寄存器有地址。

1. **选设备**：STM32 把目标设备的 CS 拉低。
2. **告诉它读哪里**：STM32 通过 MOSI 发出寄存器地址。具体如何表示“读”，由传感器协议规定。
3. **让它把数据送回来**：STM32 继续产生 SCK，传感器通过 MISO 逐位返回数据。
4. **结束**：STM32 把 CS 拉高。

SPI 每传输 **1 字节（8 位）通常需要 8 个时钟周期**。

这里有一个容易疑惑的地方：**为什么 STM32 想接收数据时，还可能需要发送一个没用的字节？**

因为主机要继续发出时钟，让传感器送出数据；在双线全双工 SPI 中，时钟继续运行时，MOSI 也会同时送出一些位。这些不需要被从机使用的填充数据常叫 **dummy byte（填充字节）**。

```text
第一个字节时间：STM32 发送寄存器地址；收到的字节可能无效
第二个字节时间：STM32 发送填充字节；传感器返回寄存器数据
```

这是普通读取流程的示意，不是所有芯片都完全相同。例如 **BMI088 的加速度计读取时，还会额外返回一个需要丢弃的无效字节**；陀螺仪的读取时序不同。编写驱动时必须查数据手册。

## 5. SPI 通信有什么特点？

**优点：**

- 通常速度较高，适合频繁读取 IMU、Flash 等设备。
- 可以同时发送和接收，通信过程直观。
- 没有固定的总线设备地址机制：通常直接用 CS 选择设备。

**局限：**

- 常见四线连接占用的 GPIO 较多。
- 多个从机通常需要各自的 CS 线。
- 主从双方必须设置兼容的时钟时序，才能正确接收数据。

## 6. CubeMX 里常见的 SPI 参数

| 参数 | 是什么意思？ |
| --- | --- |
| Master / Slave | 谁负责发起通信和产生时钟 |
| Full-Duplex | MOSI 和 MISO 分别发送、接收 |
| Baud Rate Prescaler | 把外设时钟分频，得到 SPI 时钟速度 |
| CPOL | SCK 不传数据时，默认是高电平还是低电平 |
| CPHA | 在一个时钟周期的哪个边沿读取数据 |
| Software NSS | 由程序控制 GPIO 作为 CS，而非让 SPI 外设自动管理 |

**CPOL + CPHA 决定 SPI Mode（模式 0～3）**：Mode 0～3 分别为 `(0,0)`、`(0,1)`、`(1,0)`、`(1,1)`。CPOL 决定 SCK 空闲电平，CPHA 决定在第一个还是第二个有效边沿采样。双方必须采用兼容模式，否则可能读错位。`MSB First / LSB First` 则决定每字节先传最高位还是最低位。

## 7. STM32 HAL 最小用法

- `HAL_SPI_Transmit()`：发送数据。
- `HAL_SPI_Receive()`：接收数据；主机仍会产生接收所需的时钟。
- `HAL_SPI_TransmitReceive()`：每个时钟同时发送和接收。

```c
/* 通用寄存器读取示意，不是完整、已验证的 BMI088 驱动 */
uint8_t command = reg | 0x80U;
HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, &command, 1, 10);
if (status == HAL_OK && is_bmi088_acc) {
    uint8_t discard;
    status = HAL_SPI_Receive(&hspi1, &discard, 1, 10);
}
if (status == HAL_OK) {
    status = HAL_SPI_Receive(&hspi1, data, len, 10);
}
HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
```

无论成功或失败都要把 CS 拉高；只有返回 `HAL_OK` 时才能使用接收数据。BMI088 加速度计上电默认使用 I²C，首次 SPI 通信还有接口切换要求，实际驱动必须遵循数据手册。

## 8. 常见错误与定位

- **模式或位序错误**：芯片 ID 读错；核对手册并用逻辑分析仪检查采样边沿。
- **片选错误**：选错 ACC/GYRO，或地址和数据之间提前拉高 CS。
- **时钟过快**：先降低 SPI 频率，再检查波形、接线和电压。
- **dummy byte 处理错误**：BMI088 加速度数据整体错位，需确认额外无效字节是否丢弃。
- **只检查 HAL 返回值**：`HAL_OK` 只表示 STM32 完成传输，还要验证芯片 ID、量程、单位和静止数据。

## 9. 当前 C 板工程配置

依据 `board/board.ioc`、`board/Core/Src/spi.c` 和 `gpio.c`：SPI1 为主机、双线全双工、8 bit、**Mode 0、MSB First、软件 NSS**；APB2 为 84 MHz，32 分频后 SPI 时钟为 **2.625 Mbit/s**。PB3/SCK、PB4/MISO、PA7/MOSI；PA4/ACC_CS、PB0/GYRO_CS 均为推挽输出且默认高。当前只能确认配置和代码，尚未进行上板波形验证。

参考：[STM32F407 RM0090](https://www.st.com/resource/en/reference_manual/rm0090-stm32f407-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)、[STM32F4 HAL UM1725](https://www.st.com/resource/en/user_manual/um1725-description-of-stm32f4-hal-and-lowlayer-drivers-stmicroelectronics.pdf)、[BMI088 数据手册](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)。
