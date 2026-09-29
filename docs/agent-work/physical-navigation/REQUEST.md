# Task: Implement Physical Yahboom Raspbot V2 Navigation in the Existing MicroAlchemy Project

You are working inside my existing project, `microwaferalchemy`.

This is the existing codebase for MicroAlchemy's autonomous semiconductor wafer transport system.

I now have a physical Yahboom Raspbot V2 with a Raspberry Pi 5 and a new 64 GB microSD card.

Your task is to extend the existing project to support real-world autonomous line following, tape-edge switching, graph-based navigation, and physical robot control.

**Do not create a separate project. Modify and extend the existing repository.**

Do not inspect or extract any external Yahboom ZIP files. Work from the existing repository and its current dependencies.

You may inspect existing source code, documentation, configuration files, and Git history to understand the architecture.

If existing hardware drivers are insufficient, identify the required hardware interfaces and implement them using verified documentation. Do not invent motor-controller registers or GPIO assignments.

Do not delete existing functionality, overwrite working implementations, or break existing simulations.

Implement the software, write automated tests, execute them, and prepare the codebase for deployment onto a fresh Raspberry Pi OS installation.

---

## 1. Understand the existing project

First inspect the current repository.

Identify:

- Existing Python packages and scripts.
- Motor-control implementations.
- Raspberry Pi integrations.
- Sensor-reading implementations.
- ROS 2 or simulation components, if present.
- Existing navigation algorithms.
- Existing map representations.
- Existing dependency management.
- Existing installation and deployment scripts.

Determine which components can be reused.

Preserve the existing repository structure where practical.

Add the new hardware functionality under a dedicated physical robot package, such as:

```text
microwaferalchemy/
├── hardware/
├── navigation/
├── control/
├── maps/
├── config/
├── scripts/
├── tests/
└── deployment/
```

Adapt this structure to the existing project rather than duplicating existing packages.

Create `HARDWARE_IMPLEMENTATION.md` documenting the hardware interfaces and implementation decisions.

---

## 2. Hardware platform

Target hardware:

- Yahboom Raspbot V2.
- Raspberry Pi 5.
- Four-wheel mecanum chassis.
- Four-channel infrared line-tracking sensor.
- New 64 GB microSD card.
- Raspberry Pi OS 64-bit.

Implement a hardware abstraction layer so navigation code does not directly depend on motor-driver details.

Required motor-control interface:

```python
robot.forward(speed)
robot.backward(speed)

robot.strafe_left(speed)
robot.strafe_right(speed)

robot.rotate_left(speed)
robot.rotate_right(speed)

robot.stop()
```

Use the existing project's hardware drivers where available.

Otherwise, identify the actual Yahboom motor-controller interface before implementation.

Do not guess I2C addresses, motor mixing, PWM registers, or sensor pin assignments.

Include a motor calibration utility to verify:

1. Front-left wheel.
2. Front-right wheel.
3. Rear-left wheel.
4. Rear-right wheel.
5. Forward and backward movement.
6. Left and right strafing.
7. Rotation.

Movement must never start automatically when the Raspberry Pi boots.

---

## 3. Implement four-channel edge detection

The tracking sensors are ordered from left to right:

S1, S2, S3, S4.

Use the middle sensors, S2 and S3, as the primary tape-edge detectors.

Normalize sensor values:

BLACK = 1

WHITE = 0

First implement a diagnostic program that prints all four raw sensor values.

Determine the actual sensor polarity using the hardware driver, then normalize the readings.

Implement the following states:

| S2 | S3 | State |
|---|---|---|
| BLACK | WHITE | BLACK_LEFT |
| WHITE | BLACK | BLACK_RIGHT |
| BLACK | BLACK | BOTH_BLACK |
| WHITE | WHITE | BOTH_WHITE |

Create:

```python
detect_edge(sensor_values)
```

The output should contain:

- Raw sensor values.
- Normalized values.
- Detected edge.
- Timestamp.
- Stability status.

Implement sensor debouncing.

Use the outer sensors to detect larger deviations from the expected tape position.

Both-black and both-white must be treated as ambiguous observations rather than automatically interpreted as valid edge transitions.

---

## 4. Implement continuous edge following

Create an `EdgeFollower` class.

Required interface:

```python
follow_edge(target_edge)
```

Supported target edges:

```python
BLACK_LEFT
BLACK_RIGHT
```

Implement a configurable proportional or PD feedback controller.

Use the four tracking sensors to estimate lateral tracking error and correct the robot's movement.

Initial control frequency: 20 Hz.

Make the frequency, motor speed, correction gains, and sensor thresholds configurable.

Implement:

- Continuous edge following.
- Sensor sampling.
- Motor correction.
- Edge-loss detection.
- Bounded recovery behaviour.
- Speed reduction near junctions.
- Safe motor shutdown.

The controller must work independently of the graph planner.

Provide a test script for following either edge of a straight section of black tape.

---

## 5. Implement the edge-switching algorithm

The robot must be capable of following one tape boundary and switching to the opposite boundary when instructed by the navigation system.

Create:

```python
switch_edge(current_edge, target_edge)
```

Implement this as a finite-state machine:

```text
FOLLOW_EDGE
    ↓
APPROACH_SWITCH_LOCATION
    ↓
CONFIRM_SWITCH_LOCATION
    ↓
REDUCE_SPEED
    ↓
EXECUTE_SWITCH
    ↓
SEARCH_FOR_OPPOSITE_EDGE
    ↓
VERIFY_OPPOSITE_EDGE
    ↓
RESUME_FOLLOWING
```

Because the Raspbot uses mecanum wheels, implement a controlled lateral switching mode.

Also support diagonal crossing where required by the track geometry.

The switching controller must:

- Receive authorization from the route executor.
- Confirm the robot is at the expected switching location.
- Execute a bounded crossing movement.
- Detect the opposite edge.
- Confirm multiple stable readings.
- Resume edge following.
- Stop safely if the manoeuvre fails.

Implement configurable switching distance, speed, timeout, and sensor confirmation thresholds.

Do not assume that simply detecting the opposite edge proves successful localization.

---

## 6. Calculate the minimum crossing angle

Create a geometry module for determining the minimum required switching angle.

Inputs:

```python
tape_width
sensor_spacing
sensor_detection_width
available_crossing_distance
safety_margin
robot_width
```

For a straight diagonal trajectory, calculate the initial geometric bound:

```text
minimum_angle =
atan((tape_width + 2 * safety_margin)
     / available_crossing_distance)
```

Convert the result into degrees.

Document that this calculation alone does not guarantee an unambiguous sensor transition.

Account for:

- Sensor spacing.
- Tape width.
- Sensor sampling rate.
- Switching speed.
- Sensor detection footprint.
- Available manoeuvring clearance.

Create a calibration script that tests different crossing angles on the actual robot.

Log the success rate, switching time, and final edge state.

---

## 7. Implement the track map

The physical tape network must be represented as a directed graph.

Support navigation states such as:

```text
A+
A-
B+
B-
C+
C-
D+
D-
```

Also support intermediate junction and switching nodes:

```text
B1
B2
B3
```

Use an adjacency-list representation.

Each node should contain:

```python
node_id
station_id

x
y
heading

edge_side
direction

node_type
marker_id
```

Each connection should contain:

```python
source
destination

distance_m
speed_limit_mps

action
crossing_angle_deg

enabled
```

Supported actions:

```text
FOLLOW_EDGE
SWITCH_EDGE
TURN
DOCK
STOP
```

Store the physical graph in YAML or JSON.

Make it possible to update the track map without modifying the underlying navigation algorithms.

Clearly define the meaning of positive and negative node labels in the configuration.

Do not assume that positive and negative automatically correspond to specific black/white sensor states.

Do not invent the real track topology. Create an editable example map and document the measurements required to populate the actual map.

---

## 8. Implement Dijkstra and A*

Create both algorithms.

Required interface:

```python
plan_route(
    start_node,
    destination_node,
    required_waypoints=[]
)
```

Example:

```python
route = plan_route(
    start_node="A+",
    destination_node="D-",
    required_waypoints=["B-"]
)
```

This should plan a route that starts at A+, visits B-, and terminates at D-.

Use Dijkstra as the default algorithm.

For A*, use a geometrically admissible heuristic based on the map coordinates and an appropriate maximum travel speed.

Calculate route costs using:

```text
travel_time
+ switching_time
+ docking_time
```

Return an executable route containing the required edge-following and switching actions.

Validate unreachable destinations, invalid connections, and mandatory waypoint handling.

---

## 9. Implement the route executor

Create a `RouteExecutor` class.

The graph planner should calculate the route, but the route executor should control actual robot navigation.

Implement:

```python
execute_route(route)
```

The executor must:

1. Verify the robot's current location.
2. Determine the required tape edge.
3. Begin edge following.
4. Approach the next graph node.
5. Confirm arrival.
6. Execute an edge switch if required.
7. Continue toward the next node.
8. Stop at the destination.

Do not use elapsed time alone as confirmation that the robot has arrived at a node.

If the four-channel sensor cannot uniquely recognize a junction, support an additional localization mechanism or explicit confirmation during early development.

Prepare interfaces for future QR-code or fiducial-marker localization.

Keep the planner independent of the hardware driver.

---

## 10. Safety system

Implement:

```text
IDLE
FOLLOWING
SWITCHING
DOCKING
COMPLETE
FAULT
EMERGENCY_STOP
```

Required fault conditions:

```text
LINE_LOST
SENSOR_FAILURE
SWITCH_TIMEOUT
LOCALIZATION_FAILURE
MOTOR_COMMUNICATION_FAILURE
OBSTACLE_DETECTED
```

Stop the motors whenever a critical fault occurs.

Use try/finally cleanup for hardware control.

Handle Ctrl+C.

Implement motion timeouts.

Never allow indefinite movement while searching for a missing tape edge.

Add a hardware-independent simulation or mock mode for testing navigation logic without moving the physical robot.

---

## 11. Command-line interface

Provide a working CLI.

Example commands:

```bash
waferbot sensors

waferbot motor-test

waferbot calibrate

waferbot follow --edge black-left

waferbot follow --edge black-right

waferbot switch --to black-right

waferbot map

waferbot plan --start A+ --goal D- --via B-

waferbot execute --start A+ --goal D- --via B-

waferbot stop
```

Require explicit user confirmation before physical movement.

Implement configurable speed limits.

Provide a dry-run mode for planning and testing.

---

## 12. Prepare deployment onto the new 64 GB SD card

I am developing on a MacBook and deploying onto a Raspberry Pi 5.

Create an installation process for Raspberry Pi OS 64-bit.

Generate:

```text
deployment/
    install_pi.sh
    deploy.sh
    verify_hardware.sh
    offline_install.sh
```

The scripts should:

- Install required system dependencies.
- Create a Python virtual environment.
- Install project dependencies.
- Configure required hardware interfaces.
- Install the project onto the Raspberry Pi.
- Verify motor-controller communication.
- Verify sensor communication.
- Configure logging.
- Preserve the existing software architecture.

Create a deployment workflow using SSH.

Default hostname:

```text
waferbot.local
```

Default username:

```text
pi
```

Make both configurable.

Also prepare an offline installation bundle containing the application and all dependencies that can legally and technically be bundled.

Do not assume that copying Python files directly into the visible SD card boot partition installs the application.

Do not automatically erase or reformat any storage device.

Provide separate instructions for flashing the new SD card using Raspberry Pi Imager.

---

## 13. Tests and diagnostics

Create automated tests using mocked hardware.

Test:

- All edge-detection states.
- Sensor polarity.
- Sensor failures.
- Successful edge following.
- Edge-loss recovery.
- Successful edge switching.
- Switching timeout.
- Crossing-angle calculations.
- Graph loading and validation.
- Dijkstra.
- A*.
- Mandatory waypoint routing.
- Route execution.
- Emergency stopping.

Run all tests and fix failures.

Create CSV telemetry logging.

Log sensor values, motor commands, estimated speed, target edge, detected edge, current node, target node, and fault states.

Do not report physical hardware tests as passed unless they have actually been executed on the Raspberry Pi.

---

## 14. Integrate with the existing simulation

If the existing `microwaferalchemy` project contains ROS 2, Gazebo, or a simulated wafer transport environment, preserve it.

Reuse the same graph representation and high-level navigation commands where practical.

The physical robot and simulation should share navigation logic while using different motor and sensor adapters.

Do not make ROS 2 a mandatory dependency for basic edge following on the Raspberry Pi.

---

## 15. Final deliverables

Do not stop at a design document.

Implement the source code and test suite.

After implementation, provide:

1. Summary of the existing codebase and changes.
2. Complete list of new or modified files.
3. Hardware interfaces reused or implemented.
4. Results of automated tests.
5. Any unverified hardware assumptions.
6. Installation commands for the new SD card.
7. Deployment commands from my MacBook.
8. Commands for the first sensor test.
9. Commands for the first motor test.
10. Commands for the first edge-following test.
11. Commands for the first edge-switching test.
12. Commands for executing A+ → B- → D-.

Preserve all existing MicroAlchemy functionality.

Prioritize actual hardware implementation, modularity, and safe operation over dashboards or unnecessary features.

Begin by inspecting the existing repository, then implement the system in logical stages. Continue through implementation and automated testing rather than stopping after the planning phase.