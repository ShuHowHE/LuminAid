# LuminAid

## Project Description
This smart lamp project for Arduino Uno uses a photoresistor to detect ambient light and controls an LED and a servo motor based on the light intensity. The system is toggled on/off with a button and can communicate with a PC via serial.

## Features
- Auto-adjustment of LED brightness based on ambient light.
- Servo motor control in response to light conditions.
- Real-time light intensity display on an LCD.
- Button for system toggle.
- Serial communication for remote data reading and control.

## Hardware Requirements
- Arduino Uno
- LED
- Servo Motor
- LCD Display
- Photoresistor
- Button

## Software Requirements
- Atmel Studio

## Setup and Installation
1. **Hardware Assembly**:
   - Connect the LED to the Arduino Uno's PB3 pin.
   - Connect the servo motor to the PB1 pin.
   - Attach the LCD display to the specified D0 to D7, RS, and EN pins.
   - Connect the photoresistor to the Arduino's analog input A0.
   - Connect the button to the PD2 pin.
2. **Software Setup**:
   - Install Atmel Studio.
   - Download the project code.
   - Open the project in Atmel Studio.
   - Use Atmel Studio to upload the code to the Arduino Uno.

## Usage
- **System Operation**: The system starts automatically upon powering.
- **Light Detection**: The photoresistor continuously detects ambient light levels.
- **Automatic Adjustment**:
  - In low light, the LED brightness increases, and the servo moves to a preset position.
  - In bright light, the LED dims, and the servo returns to the initial position.
- **LCD Display**: Shows the percentage of ambient light intensity in real-time.
- **Button Control**: Toggle the system on/off with the button.
- **Serial Communication**:
  - Send data from a PC to remotely monitor light intensity.
  - Send specific commands (e.g., '0') to turn off the system.

## Contributing
Contributions are welcome. Please fork the repository and submit pull requests for any enhancements.

## License
[MIT License](LICENSE.md)
