# RoboJackets Software Training - Fall 2021

The RoboJackets software training program is designed to prepare you to work on software for the RoboJackets competitive teams. This program offers practical experience writing code using C++ and ROS. It also covers some of the fundamental concepts of robotics used by all of our teams.

## Trainers

- Navaneet Kadaba (Software Training Lead)
- Nico Bartholomai
- Isaac Dale
- Hussain Gynai
- Matthew Hannay
- Charlie Jenkins
- Nicolas Leone
- Vivek Mhatre
- Adithya Vasudev

## Meeting Schedules

All new software members will attend two meetings each week. One meeting will be with their competition team. The other is a general training meeting for members of all teams. While each competition team has its own schedule for new member meetings, there are multiple sessions of the general training meetings available. These general sessions are identical and exist just to give more flexible scheduling options.

For more information about team meeting schedules and locations, check with each team's leadership.

### New Member Team Meetings
- RoboCup
  - Sundays, 7:00 - 9:30pm
- RoboNav
  - Sundays, 4:00 - 7:00pm
- RoboRacing
  - Mondays, 7:00 - 9:30pm

### General Software Training Meetings
- Wednesdays
  - 4:30 - 6:30pm
  - Van Leer, room C241
- Thursdays
  - 4:30 - 6:30pm
  - Molecular Science and Engineering Building (MoSE), room 1201A

## Resources

- [Software Training Mailing List](https://lists.gatech.edu/sympa/robojackets-training-sw)

  An email list for official training-related announcements.

- [Training Calendar](https://robojackets.org/calendar/action~month/cat_ids~12/request_format~html/)

  See details for all RoboJackets training meetings.

- [Piazza](https://piazza.com/class/ktjbkl11ngk1lr)

  Get your questions answered any time by the instructors or other students.

- [RoboJackets Training YouTube Channel](https://www.youtube.com/channel/UCh3TLV-vQzzcWGQ4u2jsMOw)

  Where all training videos will be posted.
  
- [Software Training Repository](https://github.com/RoboJackets/software-training)

  The GitHub repository that hosts most of the resources for the software training program, including project instructions and starter code.

- [STSL Repository](https://github.com/RoboJackets/stsl)

  The Software Training Support Library repository. This holds support code for the training projects. Open issues against this repository if you find bugs with the robots or simulator.

## Prerequisites

We will assume that students are familiar with the concepts covered in [AP Computer Science A](https://apstudents.collegeboard.org/courses/ap-computer-science-a). Topics we assume knowledge of will be briefly covered in Week 0 content. Topics covered in AP Computer Science A that have special syntax, properties, or behavior in C++ will be covered in the main course content.

Students should also be comfortable with math at the level of [AP Calculus AB](https://apstudents.collegeboard.org/courses/ap-calculus-ab). Derivatives and integrals will show up throughout the course.

## Topic Schedule

The content of this program is divided into three tracks: Robotics Theory, ROS, and C++. The Robotics Theory track will survey the concepts and math that make intelligent mobile robots work. The ROS track will cover how to use the Robot Operating System to program robots. The C++ track will introduce the C++ programming language, popular in robotics applications.

Week | Robotics Theory | ROS | C++
--- | --- | --- | ---
0 | | | AP CS Review
1 | Linear Algebra, Sensors, Coordinate Frames | Introduction to ROS and useful tools | Introduction to C++
2 | Computer Vision | rclcpp Basics, Timers, Topics | Classes, Inheritance, std::bind
3 | Probability, Particle Filters | Launch, Parameters | Lifetime, References, Pointers
4 | Optimization | Services | Parallelism
5 | SLAM, Mapping | TF, Custom Interfaces | Lambdas
6 | Kalman Filters | Quality of Service | Templates
7 | Control | Actions | 
8 | Path Planning | rosbag | Iterators, Algorithms

## Video Schedule

Robotics theory videos should be watched before your team-specific meetings. C++ and ROS videos should be watched before the general training meetings. You should watch the C++ videos before the ROS videos for a given week.

[Course Introduction Video](https://youtu.be/LZ4-nDEAFcY)

Week | Robotics Theory | C++ | ROS
--- | --- | --- | ---
0 | | [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha2AOCmSaLdDlBMug5XFNfwv) | 
1 | [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha2RjafLHG9lqNqZ2rzH_hdQ) | [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha1TChL2Lkm6PQQnOPRSIpDK) | [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha0y1U3yHAkCYJXXL-GiJDwF)
2 | [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha0cFU3nGomLr8cIUaKun6bl) | [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha3KemZ2wqInhNm-db8kR88r) | [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha0wxbvXIiNeEr12aoO_VX_8)
3 | [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha2ylxbALvguW15qf-mHjsGm) | [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha2BEzoEGSt-EAmx4HbvQ7RZ) | [Playlist](https://youtube.com/playlist?list=PL1R5gSylLha3YMGovXmHZGn9wVrAChkxk)
4 | | |
5 | | |
6 | | |
7 | | |
8 | | |

## Project Schedule

Each week will culminate in a programming project that uses the tools and techniques covered. The projects build on each other to produce the complete software stack needed to get a custom robot to execute a challenge game.

Week | Title |  | Description
--- | --- | --- | ---
1 | Coordinate Frame Transforms | [Instructions](projects/week_1/Instructions.md) | Transform fiducial detections from the camera's frame to the robot's body frame.
2 | Color-based Obstacle Detection | [Instructions](projects/week_2/Instructions.md) | Use HSV color detection and a projective homography to find obstacles near the robot.
3 | Particle Filter Localization | [Instructions](projects/week_3/Instructions.md) | Use a particle filter to localize the robot based on fiducial detections.
4 | Gradient Descent Optimization | | Use gradient ascent to guide the robot to the highest simulated elevation on the map.
5 | Mapping | | Build a map of the environment with a probablistic occupancy grid.
6 | Kalman Filter Tracking | | Track mineral deposits with a kalman filter.
7 | LQR Controller | | Control the robot's motion with an LQR controller.
8 | A-Star Path Planning | | Teach the robot how to avoid obstacles with the A-Star path planning algorithm.

## Robotics Theory Worksheets
Week | Title | Worksheet | Solutions
--- | --- | --- | ---
1 | Coordinate Frames, Sensors, Linear | [Sheet](https://drive.google.com/file/d/1RSI5FDaifu5RI0Zk8mrcxu_oQC9gFEAS/view?usp=sharing) | [Solutions](https://drive.google.com/file/d/1LZhwLFQjPbsGK8R7Md7tE79cBLSYIphd/view?usp=sharing)
2 | Computer Vision | [Sheet](https://drive.google.com/file/d/1WPaKruCqwO1BnkNg9nxosYZ5uVZSZLNn/view?usp=sharing) |
3 | Particle Filters | [Sheet](https://drive.google.com/file/d/1ovqS1k6tqK4Txt3FhzIUuJiRUirwYU99/view?usp=sharing) | [Solutions](https://drive.google.com/file/d/1KoD2lEKV4r5nIR7G9aXvYRyaUUf7aoA8/view?usp=sharing)
4 | | |
5 | | |
6 | | |
7 | | |
8 | | |
