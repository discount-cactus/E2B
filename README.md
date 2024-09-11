# Espresso Embedded Bus (E2B)
E2B is a variant of the 1-Wire protocol developed to be a minimum viable solution for transmitting data over a bus or to a single node with a minimal hardware footprint at the expense of slower transmission speeds.

## Usage
E2B is made to be a hardware-inexpensive option for simple data transactions between chips. Being derived from the 1-Wire protocol, all functionality with 1-Wire devices persisted so E2B can also interface with any 1-Wire compatible device.

MCU's can be connected in two ways: Directly with an interrupt pin or with a transceiver. If MCU's do not support E2B directly, a sketch included in the library allows for E2B communication through the transceiver. Hooking up the E2B line between the host and the transceiver and two UART lines between the transceiver and the MCU you wish to connect (as well as any grounding connections).

## Design considerations
### On-Board
E2B lines require a pull-up resistor - typically 4.7kohm but anywhere between 1k-10kohm range. For the host MCU, if other MCU's are connected to the E2B line, the host MCU will require the E2B line be connected to an interrupt pin. If only 1-Wire devices are connected, any digital GPIO line will suffice. Verifying all devices are connected to the same voltage supply is important to avoid accidental brown-outs or overvoltaging devices. If this is not possible, consider using level shifters. It is important to note that all connected devices be grounded together.

### Off-Board
For connecting to off-board devices, it is preferable to set the host device type to BUS (or DEFAULT) to ensure error-checking is transmitted. Most MCU's pull up E2B lines with 5V or 3.3V, so the voltage drop can be quite significant as large-gauge wires get longer. Voltage drop can be mitigated with thicker wires, impedance matching, or with additional circuitry to increase the voltage. Verifying all devices are connected to the same voltage supply is important to avoid accidental brown-outs or overvoltaging devices. If this is not possible, consider using level shifters. As with on-board devices, it is important to note that all connected devices be grounded together with solid connections.

### Secure Communication
E2B features some level of "secure" communication with devices whose secureFlag variables are set to 1. When a device is set to act as a "secure" device, it is assigned an 8-bit key. To communicate with a secured device, you must select it with select() or skip(), followed by the unlock command unlock(KEY) where KEY is the key assigned to the secured device. After unlocking the device, a single command can be sent to the device before the process must be repeated. For more details, please refer to the relevant examples provided.

<p align="center">
<img width="575" alt="Screenshot 2024-09-10 at 6 38 43 PM" src="https://github.com/user-attachments/assets/7a5153e4-368c-4415-acac-8d9ddadf340e">
</p>

### Limitations
- Because of a given device's ROM address, there can (in theory) be up to 2^64 devices connected to a given E2B line. A good rule of thumb is to keep the device count below 64 at the very most.
- The data rates, similar to that of the 1-Wire protocol, are around 11.5kbps - making this less than ideal for transmitting long and/or important messages between devices.
