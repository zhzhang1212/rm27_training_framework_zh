# 项目速查笔记

## CAN 通信核心信息流

### 初始化

```text
main()
  -> MX_CAN1_Init() / MX_CAN2_Init()
  -> ControlThread_Init()
  -> CAN_Init()
  -> registerMotor()
  -> 创建控制线程
```

### 接收反馈

```text
电机发送 0x201~0x208
  -> HAL_CAN_RxFifo0MsgPendingCallback()
  -> HAL_CAN_GetRxMessage()
  -> updateFeedback()
  -> UpdateSensorData()
  -> 更新位置、速度、电流、温度和 AliveFlag
```

### 计算并发送控制量

```text
ControlThreadEntry()
  -> AliveCheck()
  -> M2006::setOutput()
  -> sendControlData()
  -> CAN_Transmit()
  -> HAL_CAN_AddTxMessage()
  -> 电机执行
```

### 完整闭环

```text
电机反馈
  -> CAN 接收
  -> 反馈解析
  -> PID 计算
  -> 控制帧打包
  -> CAN 发送
  -> 电机执行
  -> 新反馈
```

---

## ThreadX 核心知识

### 1. ThreadX 是什么

ThreadX 是实时操作系统（RTOS）。它把程序拆成多个独立线程，并由调度器决定当前让哪个线程使用 CPU。

STM32F407 只有一个 CPU 核心，同一时刻通常只运行一个线程。多线程是通过快速切换实现的，不是所有线程真正同时执行。

### 2. 核心组成

| 模块 | 作用 |
| --- | --- |
| 内核 | 管理线程和系统资源 |
| 调度器 | 根据状态和优先级选择要运行的线程 |
| 线程 | 执行 IMU、控制、存活检测等独立任务 |
| 线程控制块 | 保存线程状态、优先级和栈信息 |
| 线程栈 | 保存局部变量、函数调用和运行现场 |
| 系统 tick | 为休眠、超时和周期任务提供时间基准 |
| 信号量/队列 | 在线程之间同步或传递数据 |

### 3. 基本工作方式

```text
main()
  -> 初始化硬件
  -> 启动 ThreadX 内核
  -> 创建内存池和线程
  -> 调度器选择最高优先级的就绪线程
  -> 线程执行任务
  -> 线程 sleep 或等待资源
  -> 调度器运行其他线程
```

线程通常写成：

```cpp
void ThreadEntry(ULONG)
{
    while (true)
    {
        DoSomething();
        tx_thread_sleep(1);
    }
}
```

无限循环中必须适当休眠或等待同步对象，否则线程可能一直占用 CPU。

### 4. 线程状态

```text
READY    已就绪，等待 CPU
RUNNING  正在运行
SLEEP    主动休眠
WAIT     等待信号量或队列
```

常见状态变化：

```text
READY -> RUNNING -> SLEEP/WAIT -> READY
```

### 5. 优先级和抢占

ThreadX 中数字越小，优先级越高：

```text
优先级 4 > 优先级 5 > 优先级 10
```

高优先级线程进入就绪状态后，可以抢占低优先级线程。高优先级线程也必须适当休眠或等待，否则低优先级线程可能无法运行。

### 6. tick 和周期

当前配置为每秒 100 tick：

```text
1 tick = 10 ms
tx_thread_sleep(1) 约等于休眠 10 ms
```

当前 control 线程每 1 tick 执行一次，频率约为 100 Hz；每 10 个周期检查一次电机在线状态，约为 100 ms。

修改线程周期后需要重新检查 PID 参数，因为当前 PID 没有显式使用 `dt`。

### 7. 初始化函数与线程入口

```text
ControlThread_Init()
  -> 只执行一次
  -> 申请线程栈
  -> 初始化 CAN
  -> 注册电机
  -> 创建线程

ControlThreadEntry()
  -> 线程创建后周期运行
  -> 在线检查
  -> PID 计算
  -> CAN 发送
  -> sleep
```

可以简单记成：

```text
Init：准备资源并创建线程
Entry：执行线程的周期任务
```

### 8. 当前项目的线程规划

| 线程 | 主要任务 |
| --- | --- |
| Control | 电机 PID、CAN 控制报文发送 |
| IMU | 初始化并读取 BMI088 |
| Alive | 使用信号量监控各线程并控制 LED 状态 |

当前只有最小 control 线程投入运行，电机在线检查暂时也在 control 线程中。IMU、独立 Alive 线程和信号量后续再实现。

### 9. 中断与线程

CAN 接收由硬件中断处理：

```text
CAN 中断更新反馈
  -> control 线程读取最新反馈
  -> PID 计算并发送控制量
```

硬件中断可以抢占线程。中断中应只做必要的快速处理，不能调用 `tx_thread_sleep()`，也不应执行长时间阻塞操作。

### 10. 开发检查清单

- [ ] 每个线程都有长期存在的控制块和栈。
- [ ] 无限循环中有 `sleep` 或同步等待。
- [ ] 明确一个 tick 对应多少毫秒。
- [ ] 数字越小，线程优先级越高。
- [ ] 中断中不执行休眠和长时间阻塞操作。
- [ ] 增加线程时同步扩大 ThreadX 内存池。
- [ ] 修改控制周期后重新检查 PID 参数。
- [ ] 注意中断和线程共享数据的一致性。
