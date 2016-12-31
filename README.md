#Electromagnetic Tether Robot

[Recorded Demo](https://www.youtube.com/watch?v=uaX5wPt5Lls)

**Overview**

Autonomous robot that is tethered by electromagnetic waves. The robot maintains a fixed distance and orientation relative to a transmitter through the use of two motors. Two LP51B circuit board loaded with embedded C / assembly routines each power the receiver, the robot itself, and the transmitter.

The robot supports the following commands:
- Move closer
- Move further
- Rotate 180 degrees
- Parallel Park
- Figure 8 maneuveur

**Project Directory**

- Robot_Receiver.c: main routine for the LP51B mounted on the robot
- Robot_Transmitter.c: main routine for the LP51B mounted on the transmitter
- ProjectSpecifications.pdf: requirement outlines and sample examples
- Report.pdf: formal report outlining investigation and design
