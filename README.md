# CS 424/524 Assignment 2 – Intelligent Mobile Robotics

## Overview

This project implements autonomous navigation and vision-based tracking on a TurtleBot using ROS.

The assignment consists of two parts:

* **Part 1:** SLAM-based mapping and autonomous navigation
* **Part 2:** Vision-based ball following using RGB-D input

---

## Part 1: Mapping and Autonomous Navigation

We first used the ROS GMapping package to build a map of the lab environment.
The robot was manually driven to explore the environment and generate a map, which was then saved using `map_server`.

Three navigation goals (L1, L2, L3) were defined in the map frame, each separated by at least 8 meters.

The robot autonomously performs the following sequence:

* Start from L1
* Navigate to L2
* Navigate to L3
* Return to L1

Navigation goals are sent programmatically via `/move_base_simple/goal`.

---

## Part 2: Vision-Based Ball Following

A vision-based controller was implemented using RGB and depth images.

### RGB Processing

* The RGB image is processed to detect the most red pixel
* The position of the red object is used to determine direction

### Depth Processing

* The depth image is used to estimate the distance to the object

### Control Strategy

* If the object is far (>1m): move forward
* If too close (<1m): move backward
* If object is left/right: rotate to align

---

## How to Run

### Part 1

```bash
roslaunch p2_zijun_li p2a.launch
```

### Part 2

```bash
roslaunch p2_zijun_li p2b.launch
```

---

## Notes

* The map file and navigation goals are configured in `config/p2a_params.yaml`
* Camera topics may vary depending on hardware setup
* Ensure all ROS nodes are running before launching navigation

---

## Repository Structure

* `launch/` – launch files
* `script/` – Python scripts
* `config/` – parameter files
* `misc/` – additional required files

---
