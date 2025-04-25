# RC5-on-STM32-without-timer

This project is an STM32 implementation of the [Arduino RC5 IR Remote Library](https://github.com/guyc/RC5), designed for **STM32 Nucleo-F411RE** using STM32CubeIDE.

The project reads **RC5 protocol** signals using **external interrupts (EXTI)** in a fully **non-blocking** manner. It includes a custom `micros()` function that emulates Arduino’s `micros()` functionality, without requiring an external hardware timer.

---

## 📦 Project Features

- ✅ Non-blocking RC5 signal decoding via **EXTI interrupts**
- ✅ RC5 protocol implementation based on the Arduino library logic
- ✅ Custom `micros()` function using only **SysTick**
- ✅ No external libraries or hardware timers required
- ✅ Developed for STM32 Nucleo-F411RE in **STM32CubeIDE**

---

## 🔧 How to Integrate in Your Own Project

To integrate this RC5 decoder into your own STM32 project:

### 1. 🧩 Configure the IR Input Pin

- Select a **GPIO pin** connected to the IR receiver (e.g. `PA0`).
- Set the **GPIO mode** to `GPIO_EXTI`.
- Enable **Interrupt trigger** for **Both rising and falling edges**.

### 2. 🔔 Enable EXTI in NVIC

- In **STM32CubeMX** or STM32CubeIDE:
  - Go to the **NVIC tab**
  - Enable the corresponding **EXTI interrupt line** (e.g. `EXTI0` for pin `PA0`).

### 3. 🧠 Copy Required Code

#### `User Code 0` section (in `main.c`)

Copy:

- `micros()` function
- All related global variables

#### `User Code 4` section (in `stm32f4xx_it.c`)

Copy:

- `EXTIx_IRQHandler()` implementation (e.g., `EXTI0_IRQHandler`)
- RC5 decoding logic based on pulse timing

Ensure that the interrupt handler matches your selected GPIO pin and EXTI line.
### 4. Change the pin label
 - in 'RC5read1' function change the pin label in 'HAL_GPIO_ReadPin' to your pin label. Here, pin has label 'IR_IN'.
---
