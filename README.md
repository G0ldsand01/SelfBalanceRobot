# SelfBalanceRobot

A self-balancing robot project.

## Key Features & Benefits

*   **Real-time Stabilization:** Utilizes sensor data (MPU6050) to maintain balance.
*   **ESP32 Microcontroller:** Leverages the ESP32's processing power for control and sensor data processing.
*   **LED Control:** Provides visual feedback through integrated LED control.
*   **Modular Design:** Well-structured code with separate files for sensor handling, LED control, and main robot logic.

## Prerequisites & Dependencies

Before you begin, ensure you have the following installed:

*   **Arduino IDE:** The Integrated Development Environment for programming the ESP32.  Download the latest version from [https://www.arduino.cc/en/software](https://www.arduino.cc/en/software).
*   **ESP32 Board Support Package:** Add the ESP32 board support to your Arduino IDE (Instructions at: [https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html)).
*   **Libraries:**
    *   **MPU6050\_simple:** Include the `mpu6050_simple.h` and `mpu6050_simple.cpp` files in your project directory. (Implementation details in the files)

## Installation & Setup Instructions

1.  **Clone the Repository:** Clone this repository to your local machine:

    ```bash
    git clone https://github.com/G0ldsand01/SelfBalanceRobot.git
    cd SelfBalanceRobot
    ```

2.  **Open the Project in Arduino IDE:** Open the `SelfBalanceRobot.ino` file in the Arduino IDE.

3.  **Install Dependencies (If Necessary):** If you encounter compilation errors related to missing libraries, install them through the Arduino Library Manager (Sketch > Include Library > Manage Libraries...).  While the provided code includes the sensor implementation, other dependencies (if any) would be installed here.

4.  **Configure Board and Port:**
    *   Select your ESP32 board from Tools > Board > ESP32 Arduino > Your ESP32 Board (e.g., "ESP32 Dev Module").
    *   Select the correct port from Tools > Port > Your ESP32 Port.

5.  **Upload the Code:** Upload the code to your ESP32 board by clicking the "Upload" button in the Arduino IDE.

## Usage Examples & API Documentation

### Example: Reading MPU6050 Data

```cpp
#include "mpu6050_simple.h"

MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  mpu.initialize();

  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  Serial.print("a/x=");
  Serial.print(ax);
  Serial.print(" a/y=");
  Serial.print(ay);
  Serial.print(" a/z=");
  Serial.print(az);
  Serial.print(" g/x=");
  Serial.print(gx);
  Serial.print(" g/y=");
  Serial.print(gy);
  Serial.print(" g/z=");
  Serial.println(gz);

  delay(10);
}
```

### API Documentation: MPU6050_simple Class

*   `MPU6050 mpu;`: Creates an instance of the MPU6050 class.
*   `mpu.initialize();`: Initializes the MPU6050 sensor.
*   `mpu.testConnection();`: Checks if the MPU6050 sensor is properly connected. Returns `true` if connected, `false` otherwise.
*   `mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);`: Reads accelerometer and gyroscope data from the MPU6050.  Populates the provided variables with the readings.

### API Documentation: LED_ESP32 Class (from LED_ESP32.h)

See `LED_ESP32.h` for API definitions if using the LED control functions.

## Configuration Options

The following options can be configured within the `SelfBalanceRobot.ino` file:

*   **MPU6050 Sensitivity:** Adjust the sensitivity of the MPU6050 by modifying the calibration values or internal configurations (Refer to MPU6050 datasheet for details).
*   **PID Controller Parameters:**  Fine-tune the PID controller parameters (Kp, Ki, Kd) for optimal balancing performance.  (Not explicitly defined in current state but assumed).
*   **Motor Control Pins:**  Define the pins connected to the motors for movement. (Not explicitly defined in current state but assumed).
*   **LED Pins:** Define the pins used to control the LEDs. (Refer to `LED_ESP32.h` and `LED_ESP32.cpp`).

## Contributing Guidelines

We welcome contributions to this project! To contribute, please follow these steps:

1.  **Fork the Repository:** Fork this repository to your GitHub account.
2.  **Create a Branch:** Create a new branch for your feature or bug fix:

    ```bash
    git checkout -b feature/your-feature-name
    ```

3.  **Make Changes:** Implement your changes and commit them with descriptive commit messages.
4.  **Submit a Pull Request:** Submit a pull request to the main branch of this repository.

## License Information

License not specified. All rights reserved.

## Acknowledgments

*   The `MPU6050_simple` class is based on community resources and examples.