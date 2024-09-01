# STM32 Project: LDC1614 Sensor Data Acquisition and UART Transmission

## Overview

This project demonstrates how to interface an STM32 microcontroller with the **LDC1614** inductance-to-digital converter using the **I2C protocol**. The acquired data is processed and transmitted over **UART**, and an onboard LED is controlled based on sensor readings.

## Features

- **I2C Communication**: Reads data from the LDC1614 sensor.
- **UART Transmission**: Transmits sensor data to a terminal via UART.
- **GPIO Control**: Turns on an LED based on sensor data thresholds.
- **Data Processing**: Combines multiple sensor readings into 16-bit values and converts them to readable formats.

## Hardware Requirements

- **STM32 Microcontroller** (e.g., STM32F4xx)
- **LDC1614 Sensor**
- **UART Interface** (for serial communication with a PC or terminal)
- **LED** (connected to GPIO pin for status indication)

## Software Requirements

- **STM32CubeIDE** (or any IDE that supports STM32 development)
- **STM32 HAL Drivers** for I2C, UART, and GPIO

## How It Works

1. **I2C Communication**: 
   The STM32 communicates with the LDC1614 sensor over the I2C bus. Sensor data is read from registers such as `DATA_MSB` and `DATA_LSB` for both channels (Channel 0 and Channel 1).

2. **Data Processing**: 
   - The MSB and LSB bytes are combined into 16-bit values.
   - These 16-bit values are converted to hexadecimal and decimal formats for easier interpretation.

3. **UART Transmission**: 
   - The processed sensor data is formatted into a string and transmitted over UART. This allows real-time monitoring of sensor readings on a connected terminal.

4. **LED Control**: 
   - If the processed sensor data exceeds a predefined threshold (310 in this example), the onboard LED is turned on. Otherwise, it remains off.

## Code Structure

### `main.c`

- **Initialization**: 
  - The HAL library is initialized, and the I2C, UART, and GPIO peripherals are configured.

- **Main Loop**:
  - The code runs in an infinite loop where it:
    - Reads data from the LDC1614 sensor using the `LDC1614_ReadRegister` function.
    - Processes the sensor data by combining MSB and LSB bytes and converting them into readable formats.
    - Transmits the sensor data via UART using the `Transmit_Data` function.
    - Toggles the onboard LED based on sensor readings.

### Key Functions

- **`LDC1614_ReadRegister(uint8_t reg, uint8_t* buffer, uint16_t size)`**:
  - Reads data from the specified LDC1614 register using I2C.

- **`Transmit_Data(uint16_t MSB_CH0, uint16_t LSB_CH0, uint16_t integerValue_MSB_CH0, uint16_t MSB_CH1, uint16_t LSB_CH1, uint16_t integerValue_MSB_CH1)`**:
  - Formats the sensor data into a string and transmits it over UART.

- **`hex_to_dec(uint16_t hex)`**:
  - Converts a hexadecimal value to a decimal integer.

### Example UART Output


### LED Behavior
- The onboard LED will turn on if the sensor data for either channel exceeds a threshold value (e.g., 310), indicating a significant change in inductance.

## Setup and Usage

### Hardware Connections
1. **LDC1614 Sensor**:
   - Connect the sensor to the STM32 I2C pins (`SCL` and `SDA`).
   - Ensure the sensor's I2C address matches the one defined in the code.

2. **UART**:
   - Connect the UART TX pin to a PC or terminal for monitoring sensor data.

3. **LED**:
   - Connect an LED to the corresponding GPIO pin (e.g., `LD2_Pin` on STM32F4xx).

### Build and Flash
1. Open the project in **STM32CubeIDE**.
2. Build the project to generate the binary.
3. Flash the binary onto the STM32 microcontroller.

### Monitor Sensor Data
- Use a terminal program (e.g., PuTTY, Tera Term) to monitor the data transmitted via UART.
- The data will be in the format shown in the "Example UART Output" section.

## Customization

- **Threshold Adjustment**: Modify the threshold value for the LED in the main loop to suit your application needs.
  
  ```c
  if (integerValue_MSB_CH0 > 310 || integerValue_MSB_CH1 > 310) {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
  } else {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  }

This `README.md` file provides an overview of the project, including features, setup instructions, and customization tips. It should help users understand how to implement and work with your STM32-based LDC1614 sensor project.
