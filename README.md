# beginner-embedded-c-pic-projects
Step-by-step Embedded C tutorials using MPLAB X, XC8, and PICSimLab. From LED blink to a full washing machine controller.

## Introduction

This repository is a beginner-friendly guide for learning embedded system design using C programming. It is tailored for those who are new to microcontrollers and want to understand how to analyze a microcontroller, work with GPIOs, and write embedded C code from scratch. Starting with the basics—like what a microcontroller is and how its pins, ports, and peripherals work—you’ll gradually learn to access and control hardware features. You'll move from simple tasks like blinking an LED to building a complete washing machine simulation project. The goal is to help you understand the thought process behind embedded development: how to plan, write, and simulate code for real-world applications.

To build and simulate our embedded systems, we’ll use only free and beginner-friendly tools. These include the [MPLAB X IDE](https://www.microchip.com/en-us/tools-resources/develop/mplab-x-ide) for development, the [XC8 Compiler](https://www.microchip.com/en-us/tools-resources/develop/microchip-xc-compilers) for building code, and [PICSimLab](https://github.com/lcgamboa/picsimlab) for simulation—no physical hardware required. We'll be working with [PIC microcontrollers](https://www.microchip.com/en-us/products/microcontrollers-and-microprocessors/8-bit-mcus/pic-mcus), mainly the **PIC16F877A**, and programming them using Embedded C, a variant of standard C tailored for hardware-level access. With just basic C knowledge, you'll be able to set up GPIOs, read switches, control LEDs, and simulate embedded behavior efficiently.

---

## 📚 Table of Contents

1. 🔑 [Prerequisites & Setup](#prerequisites--setup)   What you need before diving in, including tools and basic knowledge.

2. 💻 [Embedded C programming fundamentals](#embedded-c-programming-fundamentals)   Core C concepts tailored for embedded development.

3. 🤖 [Introduction to Embedded Systems](#-introduction-to-embedded-systems)  Understand what embedded systems are and how they power everyday devices.

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

### Practical Examples:

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
---
###  Recommended Beginner Resources

- [🔗 GreatScott! Embedded C Programming Basics – YouTube](https://www.youtube.com/watch?v=k3_fodtGmGU)
- [🔗 Arduino Bitwise Operators Explained – DroneBot Workshop](https://dronebotworkshop.com/arduino-bitwise-operators/)
- [🔗 Pointers in C – GeeksforGeeks](https://www.geeksforgeeks.org/pointers-in-c-language-set-1-introduction-arithmetic-and-array/)







