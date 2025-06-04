# Forklift: ELEX 4699 Project

The ELEX 4699 Design Project is a 4-week course at BCIT in which students must design and build a robot to solve a given problem. In May 2025, the problem involved a 4-foot square arena  with pickup and dropoff shelves for packages. The students were required to build a robot to transport the packages within the arena within a certain time period.

This GitHub contains the code and documentation for the forklift project completed by Braden Vanderwoerd and Jacob Edwards.

## Project Parameters

The shelves had three levels and were stocked with both black and white boxes. Students to deliver as many white and black boxes to respective dropoff shelves within 2.5 minutes. The packages were slightly rasied off the ground, making a forklift-type approach the most obvious and popular.

The arena included a starting zone that limited the size of the robot. It also had a birds-eye view camera mounted above that could be accessed by the robot through a TCP server socket.

Packages had ArUco markers mounted on their front faces, and dropoff shelves had markers on each package slot. The packages also had magnets set in their back faces that matched with the dropoff slots, making dropoff much easier.

The robot was required to use a Raspberry Pi as the main control unit.

## Grading Scheme

The project provided certain achievements that could be met to accumulate marks up to 100%.

*   **Teleoperation Delivery Demo (10% for first delivery, 5% for each additional)**
    * To complete a teleoperation demo, the robot must be operated remotely by a user with no physical vision of the arena. The robot must deliver as many boxes as possible within 2.5 minutes.

*   **Autonomous Delivery Demo (20% for first delivery, 10% for each additional)**
    * To complete an autonomous demo, the robot must function without any human control. The robot must deliver as many boxes as possible within 2.5 minutes.

*   **Milestones (15% each)**
    *   End of Week 1: One teleoperation delivery (no time limit)
    *   End of Week 2: One fully automatic delivery (no time limit)
    *   End of Week 4: PCB designed and built to mount on Raspberry Pi


## Solution Outline

### Hardware



## Future Considerations

