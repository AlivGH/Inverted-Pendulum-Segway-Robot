## Project Overview

The main objective of this project is to build a self-balancing robot using an inverted pendulum design. The robot is equipped with an accelerometer for position detection and DC motors for movement. The system is controlled using a PID controller implemented on an Arduino Uno.

### Features
- **Inverted Pendulum Design**: The robot maintains its balance around its equilibrium point.
- **PID Controller**: A proportional–integral–derivative controller is used for stability and control.
- **Customizable Components**: The design allows for 3D printing, laser cutting, or manual construction of the robot’s body and wheels.

## Components

- **Arduino Uno**: Controls the system using the ATmega328P microcontroller.
- **DC Motor Driver (L298N)**: Controls the speed and direction of the DC motors.
- **Accelerometer (MPU-9250)**: Measures the robot's orientation and movement.
- **DC Motors**: Provide movement and balance control.
- **Power Supply**: Multiple 9V batteries power the motors and Arduino.

## Control System

The robot’s control system is based on the kinematic equations of an inverted pendulum. The PID controller minimizes the error between the desired and actual position of the robot by adjusting motor inputs.

### Key Challenges
- **PID Tuning**: Coefficients were determined using trial and error due to limitations in simulation and hardware constraints.
- **Cost Efficiency**: Budget limitations influenced the choice of components, leading to creative solutions for achieving stability without high-cost items like encoders.

## Hardware Setup and Simulation

The final wiring and pin configuration were verified using Proteus simulation software before physical implementation. The Arduino code is provided for easy replication and customization.

## Construction and Experimentation

Two designs were developed: an initial prototype and a final optimized design. Both designs are documented with images and videos in this repository.

## How to Use

1. **Clone this repository**:
   ```bash
   git clone https://github.com/yourusername/inverted-pendulum-segway-robot.git
   ```
2. **Upload the Arduino code**: Follow the instructions in the `Arduino_Code` folder to upload the code to your Arduino Uno.
3. **Assemble the robot**: Use the schematics and construction guidelines provided in the `Documentation` folder.
4. **Run the robot**: Power the robot and observe its balancing capabilities. Adjust the PID coefficients if necessary.

## Conclusion

This project provided valuable insights into mobile robotics and control systems, particularly in tuning PID controllers and managing non-linear systems. The experience gained will be instrumental in future projects within the field of mechatronics.

## References

- Hyung-Jik Lee and S. Jung, "Control of a mobile inverted pendulum robot system," 2008 International Conference on Control, Automation and Systems, Seoul, Korea, 2008.

---

Feel free to customize the description and adjust any specific details to better fit your project.
