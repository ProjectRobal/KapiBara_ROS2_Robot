# KapiBara docker image

A docker image with a source code for KapiBara social-educational robot. It is based on Robot Operating System 2 Humble. It is split into two part:
- Robot - compose used by real robot
- Simulation - compose used for simulation in Gazebo

To run container on real hardware run:
- ./run.sh build - to build container
- ./run.sh compile - to compile all packages
- ./run.sh start - to run containter

To run simulation:
- ./sim.sh build - to build container
- ./sim.sh compile - to compile all packages
- ./sim.sh start - to run containter

## KapiBara robot:
<div align="center">
  <img src="https://github.com/user-attachments/assets/a82f75d7-b666-4903-8bbd-9410d6dbb5d1" alt="KapiBara robot" width="400">
</div>
