# Ascensor_ControlPID_Galgas
This project focuses on the implementation of gauges in conjunction with an elevator system. The configuration involves the use of an elevator equipped with a PID (Proportional-Integral-Derivative) control system.

*HOW TO WORK?*
- Elevator Control:
  This project employs digital control implemented on an Arduino Uno, with feedback from the motor's steps. This feedback is made possible by using an optical encoder to count the steps and determine the motor's direction. The primary inspiration for this project came from this video: https://www.youtube.com/embed/PEYNJVxxpMQ?si=zKexWfiABCPlgqpk
  The code includes a section for motor control, allowing precise movement of the motor by specifying the desired number of steps. This setup is especially useful when implementing a motor with a threaded rod for elevator motion. The threaded rod helps distribute the weight, relieving the motor from carrying the full load, as it only needs to move the rod.
<img src="https://github.com/JuanHoKKeR/Ascensor_ControlPID_Galgas/blob/main/Ascensor.jpg?raw=true" alt="Image Elevator" width="300">

  We use two optical encoders to capture the steps, each equipped with a 20-step wheel.
  <img src="https://github.com/JuanHoKKeR/Ascensor_ControlPID_Galgas/blob/main/Encoder.jpg?raw=true" alt="Image Encoder" width="300">

- FLOOR SELECTION WITH STRAIN GAUGES:
  To select the desired floor, strain gauges are employed as buttons. Using analog readings from the ADC ports of the Arduino, along with proper gauge conditioning through voltage dividers, we achieve a high sensitivity for our system.
<img src="https://github.com/JuanHoKKeR/Ascensor_ControlPID_Galgas/blob/main/ImagenGalgas.jpg?raw=true" alt="Galgas Imege" width="300">



<img src="https://github.com/JuanHoKKeR/Ascensor_ControlPID_Galgas/blob/main/AscensorFuncionando.jpg?raw=true" alt="Image of Work Elevator" width="300">


