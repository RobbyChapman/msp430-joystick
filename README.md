# MSP430 joystick Project

The original purpose of this project was to reverse engineer one of my computer joysticks in order to reuse the stick 
portion for [the plow](https://github.com/RobbyChapman/plow-bot). The platform for the stick consists of 1 main MCU, 
1 daughter board for the right platform buttons, and one for the MLX90333 hall effect sensor. In my case, I only want 
the stick and hall effect sensor.

![Alt text](./assets/joy1.JPG?raw=true "Original Joystick")

To get started, I dissembled the joystick and started tracing back from the 20 pin breakout on the main MCU. This 
breakout predominately consist of toggle switches and hat switches, which reside on the stick itself. These are important
as they will be reused for control of the plow-bots peripherals, for example, triggering actuator to raise.lower the 
plow, or toggling remote switches for lights etc. Below is the resulting map of traces:

    Yellow:     Front trigger
    Red:        Thumb trigger
    Brown:      Left top button
    Green:      Right top button
    Blue:       Top D-pad left
    Purple:     Top D-pad down
    Gray:       Top D-pad right
    White:      Top D-pad up

![Alt text](./assets/joy2.JPG?raw=true "GPIO Breakout")

The next step was to find which pins communicate with the MLX90333. For this task, I had to break out the scope and 
logic analyzer. The datasheet clearly defines the pinout, so it was mainly just a matter of tracing the wires from the
daughter board back to the main MCU. To do this, I soldered a few jumpers coming off the main MCU for the probes. The
resulting map for it's SPI bus looks like this(From left to right):

    Yellow:     SCK
    Blue:       SS
    red:        MOSI/MISO
    Brown:      GND
    Black:      VDD

![Alt text](./assets/joy3.JPG?raw=true "SPI Breakout")

The final step in isolating the joystick from the platform was to swap out the main MCU for my MSP430. This is where the
bulk of the work is. 

</br></br>
***
</br></br>

### **GPIO**
We have 8 buttons, and 1 hall effect sensor. The 8 buttons were re routed to the MSP as GPIO inputs. Interrupts were 
then registered for each signal. A simple continuity check was used to determine which pin services which switch.

### **SPI**
In terms of the MLX90333, the goal was to mimic the signals generated by the original MCU. To do this, I first captured 
a baseline conversation with the logic analyzer. Analysis of the resulting capture providing the timing and polarity of 
the clock. For this joystick, the clock is active hi, idle low, with captures on even clock changes for sampling the 
trailing edge of each phase. The is represented by the timing diagram below:

![Alt text](./assets/mlx90333_timing2.JPG?raw=true "Timing flow")

Where:
</br></br>
![Alt text](./assets/mlx90333_timing.JPG?raw=true "Timing definitions")

This chip is interesting in the sense that it leverages a single pin for MISO/MOSI data transfer, ultimately 
making this a half duplex device as seen below: 

![Alt text](./assets/mlx90333_frame.JPG?raw=true "Packet Frame")

This poses a few challenges for the MSP430. Basically you have to ensure whatever bytes you're shifting out, do not 
overwrite the bytes that are being shifted in, as they are share the same pin, synchronizes by the same clock. I started
solving this in software, but took a step back after a few hours as it felt like a force fit. I needed some way of 
controlling the flow of data, without eating up clock cycles toggling modes. After a bit of research, I opted to solve 
this in hardware, by adding diodes for MISO/MOSI. This way, data can come in from MISO without corrupting data going out
on MOSI and vise versa. The resulting frame generation on the MSP430 now looks like this:

![Alt text](./assets/msp430_capture.JPG?raw=true "MSP430 Capture")


### **Packet Format**
The contract for each frame takes the following format:

![Alt text](./assets/mlx90333_packet.JPG?raw=true "Packet Format")

Where the LSB is the CRC of the 6 byte X-Y-Z sum.


### **MSP430FR5994 Peripheral registers**

The MSP430 peripheral registers are multiplexed with other module functions. Each port leverages two bits used to select
the individual pins function. These two bits map to the following configuration options:

| PxSEL1    | PxSEL0    | IO Function               |
| :--------:|:---------:| :------------------------:|
| 0         | 0         | General Purpose IO        |
| 0         | 1         | Primary Module function   |
| 1         | 0         | Secondary Module Function |
| 1         | 1         | Tertiary Module Function  |


I created a small chart, given the 2 bit configuration above, based on the MSP430FR5994 datsheet:

    Port	Pin		Function		Port Select Register(1:0)
    
    1		5		UCA0CLK 			P1.1:1 P1.0:0
    1		6		UCB0MOSI			P1.1:1 P1.0:0
    1		7		UCB0MISO			P1.1:1 P1.0:0
    
    2		0		UCA0MOSI/UCA0TX		P1.1:1 P1.0:0
    2		1		UCA0MISO/UCA0RX		P1.1:1 P1.0:0
    2		2		UCB0CLK				P1.1:1 P1.0:0
    
    2		5		UCA1MOSI/UCA1TX		P1.1:1 P1.0:0
    2		6		UCA1MISO/UCA1RX		P1.1:1 P1.0:0
    2		4		UCA1CLK				P1.1:1 P1.0:0
    
    5		0		UCB1MOSI			P1.1:0 P1.0:1
    5		1		UCB1MISO			P1.1:0 P1.0:1
    5		2		UCB1CLK				P1.1:0 P1.0:1
    
    5		4		UCA2MOSI/UCA2TXD	P1.1:0 P1.0:1
    5		5		UCA2MISO/UCA2RXD	P1.1:0 P1.0:1
    5		6		UCA2CLK				P1.1:0 P1.0:1
    
    6		0		UCA3MOSI/UCA3TXD	P1.1:0 P1.0:1
    6		1		UCA3MISO/UCA3RXD	P1.1:0 P1.0:1
    6		2		UCA3CLK				P1.1:0 P1.0:1
    
    6		4		UCB3MOSI			P1.1:0 P1.0:1
    6		5		UCB3MISO			P1.1:0 P1.0:1
    6		6		UCB3CLK				P1.1:0 P1.0:1
    
    7		0		UCB2MOSI			P1.1:0 P1.0:1
    7		1		UCB2MISO			P1.1:0 P1.0:1
    7		2		UCB2CLK				P1.1:0 P1.0:1


We use this port mapping to determine which peripherals to enable. In this case, we primarily require SPI and UART. The 
following configuration is used for this project:

    JoyStick:
        Hall effect sensor:         SPI:UCB1
    Radio:
        CC1120 915MHz transceiver:  SPI:UCB3
    Display:
        Telemetry readout left:     UART:UCA1
        Telemetry readout right:    UART:UCA2
    Debug:
        Logging                     UART:UCA3
        EEPROM log                  SPI:UCB2
