# 🎵 Musical Motor Controller

An Arduino-based motor and electromagnet controller for a musical instrument, communicating with a Raspberry Pi and controlled via a web interface.

---

## 🧠 Project Overview

This system allows a user to select a musical note via a webpage.  
The selected note is sent to a Raspberry Pi, which forwards it to an Arduino over serial.  
The Arduino then:

- Moves a rotating mechanism to the correct angle using a motor with encoder feedback
- Activates or deactivates an electromagnet at the right time to trigger a sound

---

## ⚙️ Features

- ✅ PID control for precise motor positioning  
- ✅ Encoder handling using interrupts  
- ✅ Timed control of electromagnet  
- ✅ Serial communication with Raspberry Pi  
- ✅ Integration with a front-end web interface  
- ✅ Modular code (Arduino IDE compatible)  

---

## 🛠️ Components Used

- Arduino Mega 2560  
- DC Motor with encoder  
- H-Bridge (e.g. L298N)  
- Electromagnet  
- Raspberry Pi (with UART serial)  
- Web interface (HTML/JS frontend)  

---

## 🚀 How It Works

1. The user selects a musical note on a webpage.  
2. The webpage sends the note to a Raspberry Pi.  
3. The Raspberry Pi sends the note (as a character A-G) to the Arduino over serial.  
4. The Arduino calculates the correct petal angle and moves the motor using PID.  
5. The electromagnet is turned ON and OFF based on a timed cycle to strike the instrument.

---

## 🧪 Demo
<!-- If you have a demo image or GIF, upload it to /images and use: -->
<!-- ![Demo](images/demo.gif) -->

---

## 🧑‍💻 How to Run

1. Upload `MotorController.ino` to your Arduino board using the Arduino IDE.  
2. Connect the Raspberry Pi to the Arduino via serial (e.g. GPIO TX/RX or USB).  
3. Launch the HTML interface on the Pi or a browser.  
4. Select a note and watch the motor + electromagnet in action!

---

## 📜 License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.

