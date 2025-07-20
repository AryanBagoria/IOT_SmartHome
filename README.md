# Home Automation and Security System

This project implements a basic home automation and security system using a Raspberry Pi as the central controller and an Arduino as a sensor and local actuator node. The system allows for remote control of devices via a web server and sends email alerts for gas leaks or motion detection.

## Features

* **Web-Based Control:** Remotely control a fan and light connected to the Raspberry Pi via commands fetched from a web server.

* **Local Device Control:** Control an additional LED/light via commands sent from the Raspberry Pi to the Arduino.

* **Gas Leak Detection:** Arduino monitors for gas leaks using an MQ sensor and triggers a local buzzer alarm.

* **Motion Detection:** Arduino detects motion using a PIR sensor.

* **Email Alerts:** The Raspberry Pi sends email notifications for detected gas leaks or motion.

* **Serial Communication:** Seamless communication between Raspberry Pi and Arduino via USB serial.

## System Architecture

The system operates with a two-tiered architecture:

1.  **Raspberry Pi (Main Controller):**

    * Connects to the internet to fetch commands from a predefined web server URL.

    * Controls local GPIO-connected devices (Fan, Light).

    * Communicates with the Arduino via serial to send commands and receive sensor data.

    * Sends email alerts based on sensor data received from the Arduino.

2.  **Arduino (Sensor/Actuator Node):**

    * Monitors an MQ gas sensor and a PIR motion sensor.

    * Triggers a local buzzer for gas detection.

    * Controls a local LED/light based on serial commands from the Raspberry Pi.

    * Sends "G" (Gas) or "P" (Person/Motion) signals to the Raspberry Pi via serial.

## Hardware Requirements

### For Raspberry Pi:

* **Raspberry Pi Board:** (e.g., Raspberry Pi 3, 4, or Zero W)

* **USB to TTL Serial Converter (Optional but Recommended):** If connecting Arduino via GPIO serial, otherwise a standard USB cable is sufficient.

* **Relay Module:** For controlling AC devices like a fan or light via Raspberry Pi GPIO (connected to pins 23 and 24).

* **Fan & Light:** Devices to be controlled.

* **Power Supply:** For Raspberry Pi.

### For Arduino:

* **Arduino Board:** (e.g., Arduino Uno, Nano)

* **MQ-5 Gas Sensor:** Or similar gas sensor (connected to Analog Pin A7).

* **PIR Motion Sensor:** (connected to Digital Pin 2).

* **LED:** (connected to Digital Pin 3).

* **Buzzer:** (connected to Digital Pin 4).

* **USB Cable:** To connect Arduino to Raspberry Pi for serial communication and power.

## Wiring Diagram (Conceptual)

### Arduino Connections:

| NRF24L01 Pin | Arduino Pin |
| :----------- | :---------- |
| VCC | 3.3V |
| GND | GND |
| CE | 8 |
| CSN | 10 |
| SCK | 13 |
| MOSI | 11 |
| MISO | 12 |

**Note:** The SCK, MOSI, and MISO pins are the SPI pins, which are fixed on most Arduino boards (e.g., Uno: 13, 11, 12 respectively). CE and CSN pins are user-definable; in this code, they are 8 and 10.

### Motor Driver (L298N) and Motor Connections:

| L298N Pin | Arduino Pin | Description |
| :-------- | :---------- | :----------------------- |
| ENA | 3 (PWM) | Motor A Enable/Speed |
| IN1 | 4 | Motor A Direction Control |
| IN2 | 5 | Motor A Direction Control |
| IN3 | 6 | Motor B Direction Control |
| IN4 | 7 | Motor B Direction Control |
| ENB | 9 (PWM) | Motor B Enable/Speed |

Connect your two DC motors to the Motor A and Motor B output terminals on the L298N. Ensure the L298N's power input (12V or 5V depending on your motors) and ground are connected to a suitable power source.

## Software Requirements

### For Raspberry Pi:

* **Raspberry Pi OS (Raspbian):** Or any compatible Linux distribution.

* **Python 3:** Pre-installed on most Raspberry Pi OS versions.

* **Python Libraries:**

    * `pyserial`: For serial communication with Arduino.

        ```
        pip install pyserial
        ```

    * `RPi.GPIO`: For controlling Raspberry Pi GPIO pins.

        ```
        pip install RPi.GPIO
        ```

    * `urllib.request`, `smtplib`, `email.message`: These are standard Python libraries and usually come pre-installed.

### For Arduino:

* **Arduino IDE:** [Download from Arduino.cc](https://www.arduino.cc/en/software)

## Installation and Usage

### 1. Arduino Setup:

1.  **Open Arduino IDE:** Launch the Arduino IDE on your computer.

2.  **Copy Arduino Sketch:** Copy the Arduino code provided into a new sketch in the Arduino IDE.

3.  **Select Board and Port:** Go to `Tools > Board` and select your Arduino board (e.g., "Arduino Uno"). Then, go to `Tools > Port` and select the serial port connected to your Arduino.

4.  **Upload Code:** Click the "Upload" button to compile and upload the sketch to your Arduino board.

### 2. Raspberry Pi Setup:

1.  **Transfer Python Script:** Copy the Python script (`main.py` or similar) to your Raspberry Pi.

2.  **Install Python Libraries:** Open a terminal on your Raspberry Pi and install the required libraries:

    ```
    pip install pyserial RPi.GPIO
    ```

3.  **Identify Arduino Serial Port:** Connect your Arduino to the Raspberry Pi via USB. The Arduino will appear as a serial device. You can find its path using:

    ```
    ls /dev/ttyA*
    ```

    It will likely be `/dev/ttyACM0` or `/dev/ttyUSB0`. **Update the `ser=serial.Serial('/dev/ttyACM0',9600)` line in the Python script if your port is different.**

4.  **Configure Email (Important):**

    * The `SendEmail` function uses Gmail. For this to work, you need to enable "App Passwords" for your Gmail account if you have 2-Factor Authentication enabled. **Using your regular Gmail password directly is not recommended and might not work due to Google's security policies.**

    * Replace `Password = "arduinoproject"` with your generated App Password.

    * Update `Sender_Email` and `Reciever_Email` with your actual email addresses.

5.  **Web Server Setup:** Ensure your web server (e.g., `[YOUR_WEB_SERVER_URL]/IOT/getdevice.php`) is set up to return commands (`F`, `L`, `1`, `2`, `R`, `3`) in the format `#X` (where X is the command character). This project assumes you have control over this web endpoint.

### 3. Running the System:

1.  **Start Arduino:** Ensure your Arduino is powered on and running the uploaded sketch.

2.  **Start Raspberry Pi Script:** In the Raspberry Pi terminal, navigate to the directory where you saved the Python script and run it:

    ```
    python3 main.py
    ```

    You should see "Starting System..." printed to the console.

3.  **Control and Monitor:**

    * **Remote Control:** Change the command character on your web server (`getdevice.php`) to control the fan (`F`/`1`), light (`L`/`2`), and Arduino LED (`R`/`3`).

    * **Email Alerts:** Simulate a gas leak (e.g., by exposing the MQ sensor to gas) or motion (triggering the PIR) to test email notifications.

## How It Works (Detailed)

### Python Script (`main.py`)

The script initializes serial communication with the Arduino and sets up Raspberry Pi GPIO pins 23 and 24 as outputs (initially HIGH/OFF).

In an infinite loop, it performs the following:

1.  **Fetches Web Command:** It makes an HTTP GET request to `[YOUR_WEB_SERVER_URL]/IOT/getdevice.php` to retrieve a single character command (e.g., `#F`, `#L`).

2.  **Parses Command:** Extracts the command character after the `#`.

3.  **Executes Command:**

    * `F`: Sets GPIO 23 (Fan) LOW (ON).

    * `L`: Sets GPIO 24 (Light) LOW (ON).

    * `1`: Sets GPIO 23 (Fan) HIGH (OFF).

    * `2`: Sets GPIO 24 (Light) HIGH (OFF).

    * `R`: Sends character '1' over serial to Arduino.

    * `3`: Sends character '2' over serial to Arduino.

4.  **Reads Serial Data:** Checks for incoming data from the Arduino.

5.  **Sends Email Alerts:**

    * If "G" is received (from Arduino's gas sensor), it calls `SendEmail(1)` to send a "Gas Leakage Occur" email.

    * If "P" is received (from Arduino's PIR sensor), it calls `SendEmail(2)` to send a "Person/motion Detected" email.

6.  **Delay:** Pauses for 1 second before the next iteration.

### Arduino Sketch

The Arduino sketch initializes the MQ sensor (Analog A7), PIR sensor (Digital 2), LED (Digital 3), and Buzzer (Digital 4). It also sets up serial communication.

In its main loop, it performs the following:

1.  **Receives Serial Command:** Checks for incoming serial data from the Raspberry Pi. If '1' or '2' is received, it controls the LED (pin 3) accordingly.

2.  **Reads MQ Sensor:** Reads the analog value from the MQ sensor, maps it to a 0-100 range.

3.  **Reads PIR Sensor:** Reads the digital state of the PIR sensor.

4.  **Sends Alerts to Pi:**

    * If MQ5 value is greater than 50 (and a flag `f2` is set to ensure it only sends once per detection cycle), it prints "G" to serial.

    * If PIR detects motion (and a flag `f1` is set), it prints "P" to serial.

5.  **Local Gas Alarm:** If MQ5 value is greater than 50, it triggers a pulsating sound on the buzzer (pin 4). Otherwise, the buzzer is off.

6.  **Delay:** Pauses for 500 milliseconds.

## Customization

### Python Script:

* **Serial Port:** Change `/dev/ttyACM0` to your Arduino's serial port.

* **Email Addresses & Password:** Modify `Sender_Email`, `Reciever_Email`, and `Password` (use an App Password for security).

* **GPIO Pins:** Adjust `23` and `24` if your fan/light relays are connected to different Raspberry Pi GPIO pins.

* **Command Mapping:** Change the `if Comnd == "X"` logic to customize web commands and their actions.

### Arduino Sketch:

* **Sensor Pins:** Adjust `MQ`, `PIR`, `LED`, `Buzz` pin definitions if your components are wired to different pins.

* **MQ Threshold:** The `MQ5 > 50` threshold for gas detection can be adjusted based on your sensor calibration and desired sensitivity.

* **PIR Logic:** The `f1` and `f2` flags are simple debouncing/one-shot mechanisms. You might refine these for more robust sensor readings.

* **Buzzer Pattern:** Modify the `delay()` values in the buzzer section to change the alarm pattern.

* **Serial Command Actions:** Adjust the actions taken when '1' or '2' are received via serial if you want the Arduino to control different devices.

## Troubleshooting

* **Python Script Errors:**

    * **`serial.SerialException: [Errno 2] No such file or directory: '/dev/ttyACM0'`**: The serial port path is incorrect. Verify with `ls /dev/ttyA*` on your Raspberry Pi.

    * **`urllib.error.URLError`**: Problem connecting to the web server. Check your internet connection and the `LinkR` URL.

    * **`smtplib.SMTPAuthenticationError`**: Incorrect email username/password or App Password not configured correctly.

    * **`RPi.GPIO.setup` errors**: Ensure `RPi.GPIO` library is installed and you're running the script with `sudo` if necessary, or ensure your user has appropriate permissions to access GPIO.

* **Arduino Not Responding / No Serial Output:**

    * Verify Arduino is powered on and the sketch is uploaded.

    * Check serial monitor in Arduino IDE (at 9600 baud) to see if Arduino is printing "P" or "G" as expected.

    * Ensure the baud rate (9600) is consistent in both Arduino and Python code.

* **No Email Alerts:**

    * Check your `Sender_Email`, `Reciever_Email`, and `Password` in the Python script.

    * Ensure App Passwords are correctly set up for your Gmail account.

    * Check your spam folder.

* **Devices (Fan/Light/LED) Not Controlling:**

    * Double-check wiring to relays/LED.

    * Verify GPIO pins in Python script match physical connections.

    * Ensure relays are active LOW or HIGH as expected by the `GPIO.output()` calls.

    * For the Arduino LED, ensure it's receiving the correct serial commands ('1'/'2') from the Pi.

* **Buzzer Not Sounding:**

    * Check MQ sensor wiring and functionality.

    * Adjust `MQ5` threshold in Arduino code if the sensor is not triggering.

    * Verify buzzer wiring and ensure it's connected to Digital Pin 4.

## Security Note

The email password is hardcoded directly in the Python script (`Password = "arduinoproject"`). **This is a significant security risk.** For a production or more secure environment, consider using environment variables, a dedicated configuration file, or a secret management service to store sensitive credentials. For Gmail, using an [App Password](https://support.google.com/accounts/answer/185833?hl=en) is a more secure alternative to using your main account password directly.
