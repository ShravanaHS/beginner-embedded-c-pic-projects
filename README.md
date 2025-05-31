# beginner-embedded-c-pic-projects
Step-by-step Embedded C tutorials using MPLAB X, XC8, and PICSimLab. From LED blink to a full washing machine controller.

## Introduction

This repository is a beginner-friendly guide for learning embedded system design using C programming. It is tailored for those who are new to microcontrollers and want to understand how to analyze a microcontroller, work with GPIOs, and write embedded C code from scratch. Starting with the basics—like what a microcontroller is and how its pins, ports, and peripherals work—you’ll gradually learn to access and control hardware features. You'll move from simple tasks like blinking an LED to building a complete washing machine simulation project. The goal is to help you understand the thought process behind embedded development: how to plan, write, and simulate code for real-world applications.

To build and simulate our embedded systems, we’ll use only free and beginner-friendly tools. These include the [MPLAB X IDE](https://www.microchip.com/en-us/tools-resources/develop/mplab-x-ide) for development, the [XC8 Compiler](https://www.microchip.com/en-us/tools-resources/develop/microchip-xc-compilers) for building code, and [PICSimLab](https://github.com/lcgamboa/picsimlab) for simulation—no physical hardware required. We'll be working with [PIC microcontrollers](https://www.microchip.com/en-us/products/microcontrollers-and-microprocessors/8-bit-mcus/pic-mcus), mainly the **PIC16F877A**, and programming them using Embedded C, a variant of standard C tailored for hardware-level access. With just basic C knowledge, you'll be able to set up GPIOs, read switches, control LEDs, and simulate embedded behavior efficiently.

---

## 📚 Table of Contents

1. 🔑 [Prerequisites & Setup](#prerequisites--setup)   What you need before diving in, including tools and basic knowledge.

2. 💻 [Embedded C programming fundamentals](#embedded-c-programming-fundamentals)   Core C concepts tailored for embedded development.

3. 🤖 [Introduction to Embedded Systems](#introduction-to-embedded-systems)  Understand what embedded systems are and how they power everyday devices.

4. 🧠 [Microcontroller Fundamentals](#-microcontroller-fundamentals)  Key components, pinouts, and datasheets demystified.

5. ⚙️ [Inside a Microcontroller: GPIOs, Clocks & More](#-inside-a-microcontroller-gpios-clocks--more)  Learn how microcontrollers work internally.

6. 💡 [First Project: LED Blinking](#-first-project-led-blinking)  Hands-on with your very first embedded project.

7. 🌀 [Washing Machine Simulation Project](#-washing-machine-simulation-project)  Step-by-step building and simulating a real-life application.

8. 🧪 [Using PICSimLab for Simulation](#-using-picsimlab-for-simulation)  How to run your projects in a powerful simulator.

9. 🎯 [Wrap Up & Next Steps](#-wrap-up--next-steps)  Summary and recommended resources to keep learning.

---

## Prerequisites & Setup

Before diving into embedded systems design, it’s important to have some foundational knowledge and tools ready. This section covers the basics you should be comfortable with and the software you need to install.

### What You Should Know Before Starting

- **Basic C programming:**  
  Understand variables, data types, loops, conditionals, functions, and arrays. This is essential because embedded C is an extension of C language tailored for microcontrollers.

- **Basic Arduino programming (optional but helpful):**  
  Familiarity with Arduino IDE and writing simple programs helps grasp microcontroller concepts easily.

- **Basic understanding of any microcontroller:**  
  For example, learning the 8051 microcontroller basics helps you get familiar with pins, registers, and memory layout. It’s not mandatory, but useful for understanding microcontroller internals.

### Tools to Install

To follow along with projects and simulations in this repository, install the following tools on your system:

- **MPLAB X IDE:**  
  Official IDE from Microchip for PIC microcontroller development.  
  [Download Link](https://www.microchip.com/en-us/tools-resources/develop/mplab-x-ide)

- **XC8 Compiler:**  
  C compiler for PIC microcontrollers, works inside MPLAB X IDE.  
  [Download Link](https://www.microchip.com/en-us/tools-resources/develop/mplab-xc-compilers)

- **PICSimLab Simulator:**  
  A simulator for PIC microcontrollers allowing you to test your projects virtually.  
  [Download Link](https://picsimlab.com/)

- **Optional: Arduino IDE (if using Arduino for practice):**  
  [Download Link](https://www.arduino.cc/en/software)

### Summary

Make sure you have:

- Basic C programming skills  
- Optional Arduino or basic microcontroller knowledge (8051 recommended)  
- Installed MPLAB X IDE + XC8 compiler + PICSimLab simulator

Once these are set up, you are ready to start learning embedded C and microcontroller programming!

---
## Embedded C Programming Fundamentals

Embedded C is a variant of the C language tailored to program microcontrollers and embedded devices. You will use many common C features, but with a focus on controlling hardware.

### Key C Concepts Used in Embedded C (with Examples)

---

#### 1. Variables and Data Types

Store values in memory. Use specific types to match hardware sizes. In embedded systems, using precise data types like uint8_t, int16_t, etc., helps manage memory effectively and match hardware expectations.

Used to store sensor data, status flags, counters, etc.

```c
uint8_t ledState = 1;     // 8-bit unsigned integer
int temperature = 25;     // 16/32-bit signed integer
char command = 'A';       // Character type
```

#### 2. Arrays
An array is a collection of variables of the same data type, stored in contiguous memory locations.
Hold multiple values of the same type — useful for buffers or data lists.
arrays are uesd in data mapping, pwm and UART etc..
Instead of declaring five separate uint8_t variables for storing sensor values, we can use one array like:

```c
uint8_t sensorReadings[5] = {10, 20, 30, 40, 50};
```

#### 3. Loops (for, while)
Loops allow you to repeat blocks of code multiple times — a key feature in embedded systems where continuous operation is essential.
Repeat code multiple times, e.g., blinking an LED.

```c
for (int i = 0; i < 5; i++) {
    // Toggle LED here
}
```

#### 4. Conditional Statements (`if`, `else`, `switch`)

Conditional statements are used to make decisions based on certain conditions.  
If the condition is true, one task is performed; otherwise, another task is executed.

**Example:**  
If the LDR value is less than the threshold value, turn **ON** the LED; otherwise, turn **OFF** the LED.

```c
if (temperature > 30) {
    // Turn on fan
} else {
    // Turn off fan
}
```

#### 5. Functions

Functions allow you to group reusable code blocks into a single unit.  
When you need to perform a specific task multiple times, you can call the function instead of rewriting the code.

**Example:**
If you want to rotate your motor multiple times in different direction you can use functions.
```c
void turnOnLED() {
    // Code to turn on LED
}


#### 6. Pointers
Directly access memory addresses, essential for hardware registers.

```c
volatile uint8_t *portA = (volatile uint8_t *)0x05;  // Example address
*portA = 0xFF;  // Set all pins HIGH
```

#### 7. Bitwise Operators

Bitwise operators allow you to manipulate individual bits within a byte or register.  
These operators work directly on the binary representation of data, making them very useful in embedded systems, low-level programming, and situations where memory or speed is critical.

Common bitwise operators in C:

| Operator | Description                          | Example                |
|----------|----------------------------------|------------------------|
| `&`      | Bitwise AND                       | `result = a & b;`      |
| `or symbol`   | Bitwise OR                        | `result = a or symbol b;`      |
| `^`      | Bitwise XOR (exclusive OR)        | `result = a ^ b;`      |
| `~`      | Bitwise NOT (one's complement)    | `result = ~a;`         |
| `<<`     | Left shift (shifts bits left)     | `a << 2` shifts bits 2 positions left |
| `>>`     | Right shift (shifts bits right)   | `a >> 3` shifts bits 3 positions right|

#### Practical Examples:

```c
uint8_t flags = 0x00;  // 00000000 in binary

flags |= (1 << 2);     // Set bit 2  
// Explanation: (1 << 2) = 00000100  
// flags = 00000000 | 00000100 = 00000100

flags &= ~(1 << 1);    // Clear bit 1  
// Explanation: (1 << 1) = 00000010  
// ~(1 << 1) = 11111101  
// flags = 00000100 & 11111101 = 00000100 (bit 1 was already 0, no change)
```


#### 8. Preprocessor Directives and Macros

Preprocessor directives are commands that are processed **before** the actual compilation of your C code.  
They instruct the compiler to perform specific actions like defining constants, including files, or conditional compilation.


### What are Macros?

Macros are pieces of code or constants defined using the `#define` directive.  
They are replaced by the preprocessor wherever they appear in the code **before** compilation.


### Why use Macros?

- **Improve readability:** Replace hard-coded values with meaningful names.
- **Ease maintenance:** Change the value in one place rather than many.
- **Reuse code snippets:** Create inline code snippets for common tasks.
- **Conditional compilation:** Compile parts of code selectively.



### Basic Constant Macro Example

```c
#define LED_PIN 5

void turnOnLED() {
    PORTA |= (1 << LED_PIN);  // Set bit LED_PIN in PORTA to turn on LED
}
```

#### Important Embedded C Notes

- **No standard input/output**  
  Embedded systems often do **not** support standard input/output functions like `printf` by default.  
  To use these, you typically need to add specific libraries or implement serial communication protocols (e.g., UART).

- **Use the `volatile` keyword**  
  Variables that can change unexpectedly, such as hardware registers, flags set in interrupts, or shared variables in concurrent environments, **must** be declared with the `volatile` keyword.  
  This prevents the compiler from optimizing out necessary reads or writes, ensuring the program always accesses the actual hardware value.

```c
volatile uint8_t sensorFlag = 0;
```

###  Recommended Beginner Resources

- [🔗 GreatScott! Embedded C Programming Basics – YouTube](https://www.youtube.com/watch?v=k3_fodtGmGU)
- [🔗 Arduino Bitwise Operators Explained – DroneBot Workshop](https://dronebotworkshop.com/arduino-bitwise-operators/)
- [🔗 Pointers in C – GeeksforGeeks](https://www.geeksforgeeks.org/pointers-in-c-language-set-1-introduction-arithmetic-and-array/)
- and many more.

---


##   Introduction to Embedded Systems

An **embedded system** is a dedicated computing system designed to perform specific tasks it consists of both **hardware and software/firmware**. take an example of the washing machine it self controller is an embedded system that automates wash cycles based on sensor input and timing logic.

### 🧩 Why Use Embedded Systems?

Embedded systems are found everywhere—microwaves, washing machines, smart TVs, cars, and even medical equipment. They are:

- Designed for a **specific function**
- Real-time and reliable
- Built around microcontrollers (MCUs)

here in this project washing machine firmware (C code) runs on a PIC16F877A MCU, executing instructions in Embedded C to interact with sensors, motors, and switches.



### 🔧 Core Components of an Embedded System

| Component               | Description                                                                 |
|------------------------|-----------------------------------------------------------------------------|
| **Microcontroller (MCU)** | The “brain” that processes input, executes logic, and controls outputs (e.g., PIC16F877A) |
| **Input Devices**       | Buttons, water level sensors, temperature sensors, etc.                    |
| **Output Devices**      | Motors, valves, buzzers, LEDs, displays                                    |
| **Embedded Firmware**   | Code written in Embedded C that tells the MCU what to do                   |



### ⚖️ Microcontroller vs Microprocessor

| Feature              | Microcontroller                     | Microprocessor                        |
|----------------------|-------------------------------------|----------------------------------------|
| Purpose              | Task-specific control               | General-purpose computation            |
| Components           | CPU + RAM + ROM + I/O in one chip   | Only CPU, external memory/peripherals  |
| Cost                 | Low                                 | Higher                                 |
| Power Consumption    | Low                                 | High                                   |
| Example              | PIC16F877A, ATmega328P              | Intel i5, ARM Cortex-A53               |
- [Microcontroller vs Microprocessor – Simply Explained](https://www.youtube.com/watch?v=fRxAUzYxoJg)


### 🛠️ Embedded System Design Flow (Washing Machine)

1. **Define Functional Requirements**  
   - Wash, rinse, spin cycles, buzzer alerts, timers

2. **Select MCU based on requirements**  
   - PIC16F877A with enough I/O pins and timers

3. **Design Circuit**  
   - Connect input switches, water sensor, motor drivers, and buzzer

4. **Write Embedded C Code**  
   - Handle input polling, timing, state transitions

5. **Simulate using PICSimLab**  
   - Test the firmware virtually without real hardware

<br> 
</br>

<p align="center">
    General Embedded System Design Flow block diagram
  <img src="https://www.imagars.com/wp-content/uploads/2018/09/Design.Process.Embedded.System.Design.png" alt="Embedded System Design Process" width="900"/>

</p>

### 📄 Further Reading

- [📘 Sathyabama Embedded Systems Notes (PDF)](https://sist.sathyabama.ac.in/sist_coursematerial/uploads/SECA1603.pdf)
- [🔗 What is Embedded System? – Neso Academy](https://www.youtube.com/watch?v=OzT9e8Phipo)  
- [🔗 Embedded Systems in Real Life – Great Learning](https://www.youtube.com/watch?v=3e0yq5HBnpU)  


---
## 🧠 Microcontroller Fundamentals

Microcontrollers (MCUs) are the brains behind embedded systems. They are compact integrated circuits designed to perform specific control tasks in electronic systems. Understanding their basic structure and how they operate is crucial before you begin building projects like a washing machine controller.

### 🔩 What is a Microcontroller?

A **microcontroller** is a small computer on a single chip. It typically includes:

- **CPU (Central Processing Unit)** – Executes instructions.
- **Memory**  
  - **RAM** – Temporary data storage during operation.  
  - **ROM/Flash** – Stores your program code.
- **I/O Ports** – Pins to connect to external devices (like LEDs, motors, switches).
- **Peripherals** – Built-in features like timers, ADCs (analog-to-digital converters), UART (serial communication), etc.

Microcontrollers are used to **monitor and control** real-world systems based on inputs (e.g., sensors) and outputs (e.g., motors).



### 📌 Example: PIC16F877A

This 8-bit microcontroller will be used in your washing machine project. It includes:

- 40 pins with multiple I/O functions
- Built-in ADC, Timers, USART
- Flash memory to store code
- Runs at up to 20 MHz

<p align="center">
  pin diagram of PIC16F877A
  <img src="https://www.apogeeweb.net/upload/image/20201221/2020122117252429.jpg" alt="pin diagram" width="900"/>

</p>



### 📗 Reading Datasheets

Each microcontroller has a **datasheet** that explains its features, pin configuration, memory map, and register-level details.

**Why read the datasheet?**
- Know which pins are input/output capable
- Understand voltage levels, clock settings
- Use registers to configure peripherals like timers or UART

🔗 [PIC16F877A Datasheet (Microchip)](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU08/ProductDocuments/DataSheets/39582C.pdf)

The PIC16F877A Datasheet details:

- **Pin functions** (e.g., `PORTB` for GPIOs)
- **Register settings** (e.g., `TRISB` for input/output direction)
- **Peripheral configurations** (e.g., Timer0, ADC)

### 📋 Core MCU Concepts for Beginners

- **GPIO**: Digital Input/Output pins to interact with switches, LEDs, buzzers, etc.
- **Timers**: Generate delays, time events, or trigger actions periodically.
- **Interrupts**: Execute a function when an external/internal event occurs.
- **ADC (Analog to Digital Converter)**: Converts analog sensor values to digital format.
- **PWM (Pulse Width Modulation)**: Used for motor control, brightness, etc.
- **UART/USART**: For serial communication (debugging, Bluetooth, etc.)


---

### 🕹️ GPIOs (General Purpose Input/Output)

**Purpose:** Connect to LEDs, switches, or sensors.

**Registers:**

- `PORTx`: Reads/writes pin states  
- `TRISx`: Sets pin direction (`0 = output`, `1 = input`)

**Example:**

```c
TRISB = 0x00;    // Set PORTB as output
PORTB = 0x01;    // RB0 HIGH, others LOW
```

---

### ⏰ Clocks and Timing

**Clock Source:** Determines MCU speed (e.g., 4 MHz crystal)  
**Configuration:**

```c
#define _XTAL_FREQ 4000000  // Set operating frequency
```

**Timers:** Timer0, Timer1, Timer2 for generating delays or triggering events

**Example (Timer0 delay):**

```c
T0CON = 0x68;    // Timer0 on, 8-bit, prescaler 1:256
TMR0 = 0;        // Reset timer
while (!T0IF);   // Wait for overflow
T0IF = 0;        // Clear flag
```

---

### 🔄 Interrupts

**Use:** Execute code on events like a button press or sensor trigger.

**Example:**

```c
void __interrupt() isr(void) {
    if (INTCONbits.RBIF) { // PortB change interrupt
        sensorFlag = 1;
        INTCONbits.RBIF = 0; // Clear the interrupt flag
    }
}
```

---

### 📡 ADC (Analog to Digital Converter)

**Use:** Converts analog signals (e.g., from water level sensors) into digital values.

**Example:**

```c
ADCON0 = 0x41;        // Turn on ADC, select Channel 0
GO_nDONE = 1;         // Start conversion
while (GO_nDONE);     // Wait for conversion to complete
uint16_t result = (ADRESH << 8) | ADRESL; // 10-bit ADC result
```

---

### 📚 Resources

- [📄 PIC16F877A Datasheet](https://ww1.microchip.com/downloads/en/devicedoc/39582b.pdf)  
- [🎥 PIC Timers Tutorial – YouTube](https://www.youtube.com/watch?v=rbCoxZy8ODw)



### 📽️ YouTube Resources

- [🔗 Microcontrollers vs Microprocessors – Embedded Systems Basics (YouTube)](https://www.youtube.com/watch?v=lU6CLyq3YjI)
- [🔗 Microcontroller Pinout Explanation – Engineers World](https://www.youtube.com/watch?v=EkzCZn3vJpI)

---

### 📚 More Reading

- [📄 Embedded System Design PDF by Sathyabama University](https://sist.sathyabama.ac.in/sist_coursematerial/uploads/SECA1603.pdf)
- [🔗 Microcontrollers Introduction – TutorialsPoint](https://www.tutorialspoint.com/microprocessor/microcontrollers.htm)



