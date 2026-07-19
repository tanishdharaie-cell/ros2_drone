# AERODrop — DIGIPIN-Based Precision Delivery Drone

## Problem Statement

Last-mile delivery is one of the most difficult challenges in modern logistics. In cities, traffic congestion slows down deliveries. In rural and remote regions, the problem is even greater—there may be no reliable roads at all.

Villages separated by rivers, farms spread across large agricultural areas, hilly settlements, mining sites, forests, islands, and industrial campuses often require significant time and manpower for even a single delivery. In many cases, delivery personnel must travel long distances on foot or through difficult terrain, making the process slow, expensive, and sometimes impossible.

Autonomous aerial delivery offers a promising alternative, but accurate delivery requires far more than flying to a GPS coordinate. Conventional addresses are often incomplete or ambiguous, and GPS alone cannot guarantee the meter-level precision needed for safe payload delivery.

India's **DIGIPIN** introduces a standardized digital addressing system that uniquely identifies precise locations, enabling drones to navigate directly to the intended destination. However, reaching the decoded location is only part of the challenge. The drone must understand its surroundings, recognize nearby landmarks and obstacles, verify that it has reached the correct drop point, and determine whether it is safe to release the package.

Before any of this can be trusted on a real airframe, engineers need a simulation environment that faithfully reproduces quadrotor dynamics, coordinate decoding, and vision-based perception — so that flight control, waypoint navigation, and target-finding logic can all be tested safely and repeatedly.

The objective of this project is to develop a complete autonomous delivery system that combines **AeroPin (DIGIPIN-style) coordinate localization, computer vision, intelligent navigation, and autonomous mission planning** to perform safe and precise last-mile deliveries in a realistic **ROS 2 Jazzy + Gazebo Harmonic** simulation environment.

---

## The Story

A logistics company receives an order from a farmer living several kilometers away from the nearest road.

The package is small but essential—a replacement irrigation sensor needed before the evening watering cycle.

There are no proper street names.

No house numbers.

No recognizable postal address.

The customer shares only an **AeroPin**.

Within minutes, an autonomous drone is assigned the mission.

Its instruction is remarkably simple:

> **Deliver this package to this AeroPin.**

The drone lifts off from the distribution center and begins its journey across fields, rivers, uneven terrain, and isolated settlements where conventional delivery vehicles would take hours—or may never reach at all.

As it approaches the destination, the challenge changes. The decoded coordinate can only guide it approximately. Trees, electric poles, rooftops, farm equipment, and storage sheds surround the target area.

The drone now relies on its onboard cameras and a YOLO-based perception system to understand the environment. It searches the area, identifies a target object, and works out the object's *own* approximate AeroPin from what its cameras see — confirming it has found the right place before committing to a delivery.

Only after confirming both **location accuracy** and **target identification** does it complete the delivery. It then autonomously returns to its base, ready for its next mission.

No pilot intervenes.

No manual navigation is required.

The only input is an **AeroPin**—everything else is accomplished through autonomous perception, navigation, and decision-making.

> **This project is not just about automating a drone—it is about redefining last-mile logistics. By combining the location precision of AeroPin with real-time visual perception and autonomous decision-making, the drone can reliably deliver essential goods to farms, remote villages, isolated communities, and other hard-to-reach locations where conventional transportation is slow, expensive, or impractical.**

---

## Objective

Develop a complete **ROS 2 Jazzy** software stack capable of autonomously flying a simulated quadrotor delivery drone to an AeroPin-coded destination, visually searching for and identifying a target object, computing its approximate AeroPin from camera data, and completing the delivery — inside a realistic Gazebo Harmonic simulation.

**Overall pipeline:**

```
AeroPin Coordinate → Decode → Waypoint → Position Controller (PID) → Velocity Controller (PD)
   → Attitude Controller (PD) → Rotor Commands → Gazebo Harmonic Physics
   → Pose/IMU Feedback → State Machine Update
   → [Camera + YOLO Detection → Bbox Geometry → Estimated World Position → AeroPin Encode]
   → Payload Drop → Return-to-Base
```

---

## System Overview

The project consists of six major components.

### 1. Gazebo Harmonic Simulation
Provides a physically realistic environment (`sjtu_drone_description`) where rotor thrust, drag, and noise are modeled, and the drone's ground-truth pose, IMU, sonar, and dual camera feeds are all published as ROS 2 topics.

### 2. Cascaded Flight Controller
A nested **position → velocity → attitude** PID/PD control loop converts a desired (x, y, z) pose into the roll, pitch, yaw, and thrust commands the rotors need, with tunable gains and safety limits at every stage.

### 3. Teleoperation
A keyboard node that publishes velocity and takeoff/land commands manually, useful for sanity-checking flight behavior before handing control to the autonomous mission stack.

### 4. AeroPin Coordinate Codec & Mission Controller
Encodes/decodes between Gazebo local coordinates and 8-character AeroPin strings using a hierarchical 4-way subdivision scheme, then drives the drone through a takeoff → climb → goto → descend → land waypoint state machine to physically reach a decoded coordinate.

### 5. Vision-Based AeroPin Estimation
A spiral-search YOLO detection pipeline that flies the drone over the search area, detects a target object in either camera feed, estimates its real-world position from the detection geometry, and encodes that position into its own approximate AeroPin.

### 6. Payload Manager
Tracks whether the payload is currently attached and executes the drop — spawning a free-falling object in Gazebo at the drone's current position once delivery is confirmed.

---

## What You Need To Implement

This repository contains several TODOs distributed across different packages. Complete these implementations to obtain a fully autonomous AeroPin delivery and target-search drone simulation.

---

### 1. PID Controller — `sjtu_drone_description/src/pid_controller.cpp`

**TODO 1 — Implement the PID Update Step**

Compute the proportional, derivative, and integral error terms and combine them into the final control output, used identically across the position, velocity, and attitude control loops.

---

### 2. PI/PID Controller Utilities — `sjtu_drone_control/sjtu_drone_control/drone_utils/controllers.py`

**TODO 1 — Implement the `PI.compute()` Method**

Accumulate the integral term and combine it with the proportional term into a clamped control output.

**TODO 2 — Implement the `PID.compute()` Method**

Accumulate the integral term, compute the derivative from the change in error, and combine all three terms into a clamped control output.

---

### 3. Open-Loop Shape Controller — `sjtu_drone_control/sjtu_drone_control/open_loop_control.py`

**TODO 1 — Implement Task-Specific State Transitions**

Drive the drone through square, triangle, or single-direction open-loop maneuvers by advancing through `MOVE_FORWARD`/`TURN_LEFT`/`TURN_RIGHT`/`LAND` states based on elapsed time.

---

### 4. Keyboard Teleoperation — `sjtu_drone_control/sjtu_drone_control/teleop.py`

**TODO 1 — Implement Keyboard Command Dispatch**

Handle all movement, yaw, speed-adjustment, takeoff, and land keys, publishing the appropriate `Twist` or `Empty` message for each.

---

### 5. Flight Gain Tuning — `sjtu_drone_bringup/config/drone.yaml`

**TODO 1 — Tune Vertical Velocity Gains for Payload Mass**

Adjust `velocityZProportionalGain` and `velocityZDifferentialGain` so the loaded drone climbs, holds altitude, and descends smoothly with the payload attached.

**TODO 2 — Tune Thrust Headroom for Payload Mass**

Adjust `maxForce` to give the loaded drone reliable climb/hover authority without causing jerky, oversaturated thrust response.

---

### 6. AeroPin Coordinate Codec — `drone_delivery_system/drone_delivery_system/aeropin.py`

**TODO 1 — Implement `encode()`**

Implement the hierarchical 4-way subdivision that turns a Gazebo (x, y) coordinate into an 8-character AeroPin string.

**TODO 2 — Implement `decode()`**

Implement the inverse subdivision that turns an 8-character AeroPin string back into the (x, y) center of the cell it represents.

> You may take reference from Digipin by Indian Post - [Click here](https://www.indiapost.gov.in/documents/offerings/intiatives/DIGIPIN_Technical_document.pdf)
---

### 7. AeroPin Mission Controller — `drone_delivery_system/drone_delivery_system/coordinate_mission_controller.py`

**TODO 1 — Implement the CLIMB State**

Drive the drone upward to cruise altitude and transition to GOTO once stable.

**TODO 2 — Implement the GOTO State**

Steer the drone horizontally toward the decoded AeroPin target and transition to DESCEND once within range.

**TODO 3 — Implement Payload Drop Triggering**

Call the `/payload/drop` service once the drone has landed, completing the delivery.

---

### 8. Vision-Based Target Search — `drone_delivery_system/drone_delivery_system/yolo_search_controller.py`

**TODO 1 — Generate the Spiral Search Pattern**

Build an expanding square-spiral list of waypoints used to sweep the search area for the target object.

**TODO 2 — Estimate World Position from Camera Detections**

Convert a YOLO bounding box from either the bottom (straight-down) or front (forward-facing) camera into an estimated real-world (x, y) position, which is then encoded into the target's approximate AeroPin.

---

## Running the Project

**Pre-requisite packages:**
```bash
sudo apt update

sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-rviz2 \
  ros-jazzy-teleop-twist-keyboard \
  xterm
```

**Before opening any terminal, build the workspace:**
```bash
cd ~/ros2_drone
colcon build
```
> If you encounter gazebo crash while building description package
> ```bash
> colcon build --packages-select sjtu_drone_description --cmake-args -DBUILD_TESTING=OFF
> ``` 

**For every new terminal, source the workspace:**
```bash
source install/setup.bash
```

---

### Step 1 — Launch the Simulation

Open a terminal and run:

```bash
ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py
```

This single command:

* Starts Gazebo Harmonic (`gz sim`) with the drone world
* Spawns the quadrotor model with the `plugin_drone` cascaded PID controller
* Opens an xterm teleop window for optional manual keyboard control
* Launches RViz2 with the drone sensor visualization preset

Wait until Gazebo fully loads and the drone model appears in the simulation before running Step 2.

---

### Step 2 — Fly the Drone Manually (optional sanity check)

Using the xterm teleop window opened in Step 1, or in a new terminal:

```bash
ros2 run sjtu_drone_control teleop
```

Take off, fly around, and land to confirm the flight controller is stable before trusting it with a mission.

---

### Step 3 — Run the AeroPin Mission Controller

Open a new terminal (workspace already sourced via `.bashrc`) and run:

```bash
ros2 run drone_delivery_system coordinate_mission_controller
```

Enter an AeroPin coordinate when prompted. This starts the AeroPin node, which will autonomously:

1. Enable position control mode on the drone (`/simple_drone/posctrl → true`)
2. Publish `/simple_drone/takeoff` to command autonomous takeoff
3. Climb to cruise altitude and navigate to the decoded AeroPin coordinate
4. Descend, land, and trigger the payload drop at the target
5. Confirm mission completion

---

### Step 4 — Run the Vision-Based Target Search

Open a new terminal and run:

```bash
ros2 run drone_delivery_system yolo_search_controller
```

Enter a YOLO object class when prompted (e.g. `person`, `car`, `dog`). The drone will take off, fly an expanding spiral search pattern, run YOLO on both cameras, and report the approximate AeroPin of the detected target once found.

---

## Bonus Challenges

Once you've completed all the required TODOs and the drone can successfully complete an AeroPin delivery and a target search, try extending the project with the following challenges.

---

## Bonus Challenge 1 — Multi-Drop Delivery Run

### Objective

Currently, the drone delivers to a single AeroPin coordinate before returning to base.

Modify the project so the drone accepts a **list** of AeroPin coordinates and autonomously delivers to each one in sequence, returning home only after the last drop.

### What to implement

Create or extend a node that:
- Loads a list of AeroPin coordinates (from a parameter, YAML file, or service call).
- Sends the drone to each coordinate in turn via the mission controller.
- Confirms each drop before advancing to the next waypoint.
- Returns to the launch pad after the final delivery.
- Reports a summary (succeeded/failed per waypoint) at the end of the run.

## Bonus Challenge 2 — Collision Detection & Orientation Control

### Objective

Currently, the drone flies directly to each AeroPin coordinate without considering nearby obstacles or maintaining a desired heading.
Modify the project so the drone detects obstacles in real time, performs collision avoidance when necessary, and continuously maintains the correct orientation throughout the mission.

### What to implement

Create or extend a node that:

- Detect obstacles within a configurable safety radius and classify collision risk.
- Triggers an avoidance maneuver (hover, stop, or reroute) when an obstacle enters the critical zone, then resumes the original mission once the path is clear.
- Computes and maintains the drone's yaw toward its next waypoint while rejecting unsafe attitude commands before sending them to the flight controller.
  
---

## Deliverables

### 1. Source Code
- Completed implementations for all TODOs
- Functional ROS 2 packages
- Updated launch and configuration files

### 2. Demonstration Video

Show:
- Gazebo Harmonic simulation
- Manual teleoperation
- Cascaded PID/PD flight control
- AeroPin coordinate decoding and waypoint navigation
- Vision-based spiral search and target AeroPin estimation
- Payload drop and return-to-base
- A complete autonomous delivery run

### 3. Report

Briefly describe:
- Cascaded PID/PD controller implementation
- AeroPin encode/decode algorithm
- AeroPin waypoint state machine design
- Vision-based world-position estimation approach
- Payload drop and mission-completion logic
- Challenges encountered

---

## Final Message

Delivery drones can't rely on a pilot's judgment, a clear line of sight, or a second chance if a rotor falters mid-flight. Every successful autonomous delivery depends on the seamless integration of stable flight control, precise coordinate-based navigation, and reliable visual perception.

By completing this project, you will implement each stage of that pipeline — from rotor-level PID control to AeroPin coordinate decoding to vision-based target localization — and gain practical experience with the same ROS 2 and Gazebo technologies used in real aerial robotics development.

The objective is not simply to fly a simulated quadrotor across a yard, but to understand how an aerial robot turns a single coded coordinate into a safe, confirmed, fully autonomous delivery — and how it can in turn use its own eyes to derive a coordinate for what it finds.

## Final Report
The complete project report — covering flight controller design, AeroPin algorithm, state machine design, vision-based estimation, payload logic, and challenges encountered — is available at [`docs/AERODrop_Final_Project_Report.docx`](docs/AERODrop_Final_Project_Report.docx).
