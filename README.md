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

```
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
The **PIC16F877A** is a very popular 8-bit microcontroller, like a tiny computer on a single chip. It's known for being versatile and comes with many built-in "tools" (peripherals). It has a special design (Harvard architecture), understands just 35 simple instructions, and can run really fast, up to 20 million operations per second (20 MHz). Inside, it has a central brain (CPU), different types of memory (Flash for your program, RAM for temporary data, EEPROM for saving settings), and many integrated "tools" that help it do complex jobs easily.

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

**What they are:** Imagine your microcontroller has many "digital doors" or "switches." These are GPIO pins. You can configure each "door" to be either:
* **Input:** To read if a button is pressed, or if a sensor sends a HIGH/LOW signal.
* **Output:** To turn an LED on/off, or control a motor.

**How the PIC16F877A uses them:** The PIC16F877A organizes its GPIO pins into groups called "Ports" (like PORTA, PORTB, PORTC, PORTD, PORTE). You use special internal registers to tell the microcontroller whether each pin should be an input or an output, and then to read or set its state.

* **Key Registers:**
    * `TRISx`: This register controls the **direction** of pins in Port `x`. Setting a bit to `0` makes the corresponding pin an **output**; setting it to `1` makes it an **input**.
    * `PORTx`: This register is used to **read** the state of input pins or **write** a logic level (HIGH/LOW) to output pins.
    * `LATx`: (Often used for outputs on older PICs) This is a "latch" register that holds the output value. Using `LATx` for writing outputs can help avoid a common issue called "Read-Modify-Write."

* **Code Examples (single line access):**
    * Set all pins of PORTB as outputs: `TRISB = 0x00;`
    * Set pin RA0 (Port A, Pin 0) as an input: `TRISAbits.RA0 = 1;`
    * Turn on all LEDs connected to PORTD: `PORTD = 0xFF;`
    * Set pin RB1 (Port B, Pin 1) to a HIGH logic level: `PORTBbits.RB1 = 1;`
    * Check if pin RC2 (Port C, Pin 2) is LOW: `if (PORTCbits.RC2 == 0) { /* Pin is LOW */ }`

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

**What they are:** Just like your heart beats to keep you alive, a microcontroller needs a "clock" to keep its operations synchronized. This clock determines how fast the microcontroller executes instructions. "Timers" are like built-in stopwatches or alarm clocks. They can count time, measure durations, or trigger events after a specific period without needing the main program to constantly check.

**How the PIC16F877A uses them:** The PIC16F877A usually uses an external crystal (like a tiny quartz clock) to generate its main clock signal. It has several internal timers (Timer0, Timer1, Timer2) that you can configure for various timing tasks.

* **Clock Frequency Definition:**
    * Tell the compiler your crystal's frequency (e.g., 20 MHz) so delay functions work correctly: `#define _XTAL_FREQ 20000000`
* **Timer Configuration (Example: Timer0 for basic timing):**
    * Enable Timer0 in 8-bit mode: `T0CON = 0xC0;`
    * Start/Stop Timer0: `T0CONbits.TMR0ON = 1;` / `T0CONbits.TMR0ON = 0;`
    * Reset the Timer0 count: `TMR0 = 0;`
    * Check if Timer0 has "overflowed" (reached its maximum count): `if (INTCONbits.TMR0IF) { /* Overflow occurred */ }`
* **Delay Function:**
    * Pause the program for 100 milliseconds: `__delay_ms(100);`
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
**What they are:** Think of interrupts like a doorbell for your microcontroller. Normally, your microcontroller is busy doing its main job (like running the washing machine cycle). But if something important happens – like a button being pressed, a sensor reaching a certain value, or a timer finishing its count – an interrupt can "ring the doorbell." The microcontroller then immediately pauses its current task, goes to a special "Interrupt Service Routine" (ISR) to handle the urgent event, and then returns to its original task as if nothing happened. This makes your system much more responsive.

**How the PIC16F877A uses them:** The PIC16F877A has various interrupt sources (e.g., external pin changes, timer overflows, ADC completion). You need to enable global interrupts, peripheral interrupts, and specific interrupt sources.

* **Code Examples (single line access):**
    * Enable **Global Interrupts** (the main switch for all interrupts): `INTCONbits.GIE = 1;`
    * Enable **Peripheral Interrupts** (for things like timers, ADC, communication): `INTCONbits.PEIE = 1;`
    * Enable the **Port B Change Interrupt** (triggers when any pin on Port B changes state): `INTCONbits.RBIE = 1;`
    * Clear the Port B Change Interrupt Flag (important to do inside the ISR after handling the event): `INTCONbits.RBIF = 0;`
      
* **Interrupt Service Routine (ISR) structure:**
    ```c
    void __interrupt() my_isr(void) { // This special function runs when an interrupt occurs
        if (INTCONbits.RBIF) {      // Check if the Port B change interrupt caused this
            // Code to handle the button press or sensor change on Port B
            INTCONbits.RBIF = 0;    // Clear the interrupt flag so it can trigger again
        }
        // You can add 'else if' statements here to check for other interrupt flags
    }
    ```


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
**What it is:** The real world is full of "analog" signals – things that change smoothly, like the brightness of light, the exact temperature, or the water level in a tank. Microcontrollers, however, understand only "digital" signals (ON/OFF, 0s and 1s). An Analog-to-Digital Converter (ADC) is a special tool inside the microcontroller that takes an analog voltage and converts it into a digital number that the microcontroller can understand and use.

**How the PIC16F877A uses it:** The PIC16F877A has a 10-bit ADC, meaning it can convert an analog signal into one of 1024 different digital values (from 0 to 1023). You can select which pin (channel) to read the analog signal from.

* **Code Examples (single line access):**
    * Configure pin AN0 (Port A, Pin 0) to be used as an analog input: `ADCON1 = 0x8E;` (This also sets voltage references for the ADC).
    * Select ADC Channel 0 to read from: `ADCON0bits.CHS = 0;`
    * Start the ADC conversion: `ADCON0bits.GO_nDONE = 1;`
    * Read the complete 10-bit digital result from the ADC: `uint16_t adc_value = (ADRESH << 8) | ADRESL;` (The result is stored in two 8-bit registers, `ADRESH` and `ADRESL`, which are combined to form the 10-bit value).
**Use:** Converts analog signals (e.g., from water level sensors) into digital values.

**Example:**

```c
ADCON0 = 0x41;        // Turn on ADC, select Channel 0
GO_nDONE = 1;         // Start conversion
while (GO_nDONE);     // Wait for conversion to complete
uint16_t result = (ADRESH << 8) | ADRESL; // 10-bit ADC result
```
### 💡 Pulse Width Modulation (PWM)
**What it is:** PWM is a clever trick to control "analog-like" behavior using only digital ON/OFF pulses. Imagine rapidly flicking a light switch on and off. If you turn it on for a longer percentage of the time (higher "duty cycle"), the light appears brighter. If you turn it on for a shorter percentage, it appears dimmer. This technique is widely used for controlling the speed of motors, dimming LEDs, or generating analog voltages.

**How the PIC16F877A uses it:** The PIC16F877A has special "Capture/Compare/PWM" (CCP) modules that can generate these precise PWM signals. You configure the frequency (how fast the pulses repeat) and the duty cycle (the percentage of time the pulse is ON).

* **Code Examples (single line access for CCP1):**
    * Configure CCP1 module for PWM mode: `CCP1CON = 0x0C;`
    * Set the PWM period (which determines the frequency) using Timer2: `PR2 = 249;` (This value, along with the clock and prescaler, sets the PWM frequency, e.g., to 5kHz with a 20MHz crystal).
    * Set the PWM duty cycle (e.g., to 50%): `CCPR1L = 124;` (This value, in combination with other bits, sets the "on" time of the pulse).
    * Enable Timer2 (which is often used as the time base for PWM): `T2CONbits.TMR2ON = 1;`

---


### 💾 EEPROM (Electrically Erasable Programmable Read-Only Memory)

**What it is:** EEPROM is like a small, special notebook inside the microcontroller where you can write down important settings or data that you want to remember even when the power is turned off. Unlike RAM (which forgets everything when power is lost), EEPROM keeps its data permanently until you specifically erase or rewrite it. This is perfect for storing things like user preferences, calibration values, or a device's last known state.

**How the PIC16F877A uses it:** The PIC16F877A has 256 bytes of EEPROM data memory. Accessing it involves a specific sequence of steps (setting an address, writing data, and then initiating the write/read operation).

* **Code Examples (conceptual single line access - actual access involves a sequence of steps):**
    * Set the EEPROM memory address to write/read from: `EEADR = 0x00;`
    * Write a byte of data (e.g., `0xAA`) to the EEPROM (this is part of a larger write sequence): `EEDATA = 0xAA;`
    * Initiate a read operation from EEPROM: `EECON1bits.RD = 1;`
    * Read the data byte from EEPROM after a read operation: `uint8_t data = EEDATA;`

### 📡 Communication Protocols (UART, SPI, I2C)

**What they are:** Microcontrollers often don't work alone. They need to "talk" to other devices, like sensors, displays, other microcontrollers, or even a computer. Communication protocols are like different languages or rules that devices follow to send and receive data reliably.

**How the PIC16F877A uses them:** The PIC16F877A supports several common serial communication protocols through its built-in modules:

* **UART (Universal Asynchronous Receiver/Transmitter):**
    * **What it is:** A very common and relatively simple way for two devices to communicate using just two wires (one for sending, one for receiving). It's "asynchronous" because there's no shared clock signal between the devices; they rely on agreed-upon speeds (baud rates). Often used for debugging (sending data to a PC terminal) or communicating with modules like Bluetooth.
    * **Code Examples (single line access):**
        * Enable UART Transmit: `TXSTAbits.TXEN = 1;`
        * Enable the overall Serial Port: `RCSTAbits.SPEN = 1;`
        * Transmit a single character: `TXREG = 'A';`
        * Wait for and receive a character: `while(!RCSTAbits.RCIF); char received_char = RCREG;`

* **SPI (Serial Peripheral Interface):**
    * **What it is:** A faster, synchronous serial communication protocol that uses more wires (typically 4). It's "synchronous" because it uses a shared clock signal, making it reliable for higher-speed data transfer. Often used for communicating with fast peripherals like SD cards, external ADCs, or some types of displays.
    * **Code Examples (single line access):**
        * Configure the SSP (Synchronous Serial Port) module for SPI Master mode: `SSPCON = 0x20;` (Master mode, Fosc/4 clock speed)
        * Transmit a byte of data: `SSPBUF = 0x55;`
        * Wait until transmission/reception is complete: `while(!SSPSTATbits.BF);`

* **I2C (Inter-Integrated Circuit):**
    * **What it is:** A two-wire serial bus (SDA for data, SCL for clock) that allows multiple devices to communicate with each other using a "master-slave" relationship. It's great for connecting many low-speed devices (like sensors, small EEPROMs, or real-time clocks) to a single microcontroller using minimal pins.
    * **Code Examples (single line access):**
        * Configure the SSP module for I2C Master mode: `SSPCON = 0x28;` (Master mode, Fosc/4 clock speed)
        * Generate an I2C Start condition (to begin communication): `SSPCON2bits.SEN = 1;`
        * Generate an I2C Stop condition (to end communication): `SSPCON2bits.PEN = 1;`

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



