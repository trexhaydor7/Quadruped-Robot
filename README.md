# Quadruped-Robot
A servo-driven robotic dog controlled by a Raspberry Pi for smooth movement and intelligent autonomous behavior. Along with an intent to implement robust algorithims and mechnanical design, this project is aimed to be open-sourced, well-documented, and meant to spark new ideas in others.

<div align="center">
  <img src="assets/quadruped_leg_render.png" alt="Quadruped leg design" width="500"/>
</div>

# Technical Details

This project includes two major software components, a custom CLI interface, named QGC (Quadruped Guidance Computer), and a trained AI model using the Jetson Nano Orion. At the time of writing, neither of these are fully implemented, but I have plans to implement them in the future due to my interest in low-level programming, AI models, and embedded systems engineering. Furthermore, all CAD for the robot was designed by me in Fusion 360 and was printed using on the Bambu Lab H2S using either generic PLA or PETG HF (for particularly critical parts where strength is needed). Other than 3D-printed parts, all parts were ordered from a varity of suppliers, however many Amazon. The electronics of the quadruped robot are composed of the Jetson Orion Nano for high-speed computation and high-level instructions, and the low-level custom PCB that serves as a PWM-driver for all 12 Hi-Torque MG996R servo motors. 
