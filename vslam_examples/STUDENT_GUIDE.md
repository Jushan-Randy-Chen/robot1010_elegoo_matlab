# Student Guide: Map and Position with the ELEGOO Smart Robot Car V4

ROBT 1010 Intro to Robotics Engineering, RPI.
This guide uses ASD-STE100 Simplified Technical English.

**Contents**

1. [Before you start](#1-before-you-start)
2. [How to read the figures](#2-how-to-read-the-figures)
3. [Task 1: Read the sensors](#3-task-1-read-the-sensors)
4. [Task 2: Calibrate the motion](#4-task-2-calibrate-the-motion)
5. [Task 3: Make a sonar map from one position](#5-task-3-make-a-sonar-map-from-one-position)
6. [Task 4: Move in a square, calculate the position and make a map](#6-task-4-move-in-a-square-calculate-the-position-and-make-a-map)
7. [Task 5: Make a camera map with visual SLAM](#7-task-5-make-a-camera-map-with-visual-slam)
8. [Record your results](#8-record-your-results)
9. [Words that this guide uses](#9-words-that-this-guide-uses)

---

## 1. Before you start

### 1.1 Purpose

A mobile robot must know two things:

- **Localization:** Where am I? The robot calculates its position and its heading.
- **Mapping:** What is near me? The robot makes a map of the objects near it.

This car has no GPS and no wheel encoders. It uses its sensors for these two functions. Each task adds one new idea to the task before it.

| Task | New idea | Sensors | Time (simulator) |
|---|---|---|---|
| 1 | What each sensor measures | sonar, camera, gyro | 20 s |
| 2 | Calibration: measure the motion of the car | sonar, camera | 30 s |
| 3 | Mapping when the position is known | sonar, pan servo | 15 s |
| 4 | Mapping when the position is calculated (dead reckoning) | sonar, camera, pan servo | 50 s |
| 5 | Visual SLAM: mapping and localization at the same time | camera, sonar | 5 to 8 min |

### 1.2 What you must have

- MATLAB R2024a or later (tested with R2026b), with the Computer Vision Toolbox and the Image Processing Toolbox.
- The files of this kit in one folder. Do not move or rename the files.
- For the real car: a charged ELEGOO Smart Robot Car V4 with the course firmware, a tape measure and 1 m of clear floor (2.5 m for task 5).

### 1.3 How to run a task

1. Set the MATLAB current folder to the folder of this kit.
2. Open `elegooMappingTemplate.m`.
3. Set `TASK` to the task number (1 to 5).
4. Set `USE_SIMULATOR = true`. Do each task in the simulator first.
5. Click **Run** (or push F5).
6. Read the Command Window. Look at the figures. Answer the questions.
7. Do the same task on the real car:
   1. Set the power switch of the car to ON.
   2. Connect the computer to the Wi-Fi network of the car (`ELEGOO-...`).
   3. Set `USE_SIMULATOR = false`.
   4. Click **Run**.

> **CAUTION:** KEEP THE CAR AWAY FROM STAIRS AND TABLE EDGES. THE CAR CAN FALL AND BREAK.
>
> **CAUTION:** TO STOP THE CAR, PUSH CTRL+C IN THE COMMAND WINDOW. IF MATLAB STOPS, THE MOTORS STOP AFTER 1 SECOND.

> **NOTE:** The simulator car and the real car are not the same. Do task 2 on the real car before you do task 3, 4 or 5 on the real car.

---

## 2. How to read the figures

Each figure has a label at the top: **Task N, Figure N.k: title**.

- **N** is the task number. **k** is the figure number in that task.
- The line below the label shows the source of the data: **simulator** or **real car**.
- A figure can have more than one panel. Each panel has a letter: **(a)**, **(b)**, **(c)**, **(d)**.
- The legend below or next to each panel tells you what each color and line shows.
- Items with **(simulator only)** are the true values. The simulator knows the true room and the true path. The real car does not. On the real car, use a tape measure.

The task also writes its results in the Command Window and in the variable `result` in the Workspace.

---

## 3. Task 1: Read the sensors

**Purpose:** Find what each sensor measures, its range and its noise.

**What the car does:** The car stays at one position for 10 seconds. Then it takes one photo. Then it turns left for 0.8 seconds.

**Procedure:**

1. Set `TASK = 1`. Click **Run**.
2. During the first 10 seconds, move your hand in front of the sonar. Move it near the sensor and then away. (In the simulator, the sonar measures the room.)

### Figure 1.1: Sensor readings

![Task 1, Figure 1.1: Sensor readings](docs/images/task1_figure1_1.png)

**How to read Figure 1.1:**

- **(a) Sonar distance:** the distance to the nearest object in front of the car, for 10 seconds. A flat line means that nothing moved. The small changes in the line are noise.
- **(b) Camera photo:** each green **+** is a corner that the computer found. Visual SLAM (task 5) uses these corners. Areas with no texture have no corners.
- **(c) Gyro turn rate:** how fast the car turns, while it turns left.
- **(d) Gyro angle:** the sum of (turn rate x time). This is the angle that the car turned.

**Expected results in the simulator:** the sonar shows approximately 115 cm. The camera finds approximately 350 to 400 corners. In the simulator, the gyro units are deg/s. The units of the real gyro are not known. Question f helps you find them.

**Questions:**

- a. Put a book 30 cm in front of the sonar. Use a ruler. What value does the sonar show?
- b. What value does the sonar show when no object is in front of it?
- c. Turn the car until the sonar is at an angle of more than 45 degrees to a wall. What occurs? Why?
- d. Turn the camera to a blank wall, then to a bookshelf. How many corners does the computer find each time?
- e. Is the gyro value positive or negative when the car turns left?
- f. Measure the angle that the real car turned. Compare it with the gyro angle in panel (d). What are the units of the gyro?

---

## 4. Task 2: Calibrate the motion

**Purpose:** The car has no wheel encoders. To calculate its position, it counts its steps. This task measures the length of one forward step and the angle of one turn step.

**What the car does:**

1. It turns until the sonar finds an object 70 cm to 130 cm away.
2. It moves 8 steps forward and 8 steps back. The change of the sonar distance gives the length of one step.
3. It turns in place approximately 400 degrees, one step at a time. The camera measures each turn step.

**Procedure:**

1. Put the car approximately 1 m from a wall or a large box.
2. Set `TASK = 2`. Click **Run**.
3. Read the two results in the Command Window:
   ```
   Write these values in SETTINGS:
     S.cmPerStep  = 5.2;
     S.degPerTurn = 16.1;
   ```
4. Write your two values in the SETTINGS section of the template.

### Figure 2.1: Turn calibration with the camera

![Task 2, Figure 2.1: Turn calibration with the camera](docs/images/task2_figure2_1.png)

**How to read Figure 2.1:**

- The line below the label shows the two results.
- **(a) Turn angle from the camera:** the sum of the turn angles that the camera measured. The dashed line is 360 degrees (one full turn).
- **(b) Matches with the first photo:** the number of corners that match the first photo. The number is high at the start and when the car points in the first direction again. The red **\*** shows that step. At this step, the car turned 360 degrees.
- Why two methods? The sum in panel (a) adds a small error at each step. The match in panel (b) does not add errors. The result uses panel (b).

**Expected results in the simulator:** one forward step = 5.0 cm to 5.2 cm. One turn step = 16 to 18 degrees. The result changes a little each time.

**Questions:**

- a. Do the task three times. How much do the results change? Why?
- b. Is one step back the same length as one step forward?
- c. Do the turn steps change more than the forward steps? Why? (Help: the four wheels slide to the side when the car turns.)
- d. Do the results change on a carpet? On a tile floor?

---

## 5. Task 3: Make a sonar map from one position

**Purpose:** Make a map when the position of the car is known. The car does not move, so its position is known.

**What the car does:** The car does not move. The pan servo turns the sonar from the right to the left. The sonar measures the distance at each 5 degrees. This is a **scan**. Each value gives one point:

- x = distance x cos(angle)
- y = distance x sin(angle)

**Procedure:**

1. Put the car less than 1 m from walls, boxes or furniture. The sonar range is approximately 150 cm.
2. Set `TASK = 3`. Click **Run**.
3. At the start, the sensor head turns to pan(150) for 1 second. Look at the head. If it turns to the RIGHT, set `S.panLeftIsBigger = false` in SETTINGS. Then do the task again.

### Figure 3.1: Sonar map from one position

![Task 3, Figure 3.1: Sonar map from one position](docs/images/task3_figure3_1.png)

**How to read Figure 3.1:**

- The car (blue triangle) is at (0, 0). It points to +x (forward). +y is to the left of the car.
- **(a) Scan points:** each red dot is one sonar echo, that is, an object. Directions with no echo have no dot. The green lines are the true walls and furniture (simulator only).
- **(b) Occupancy grid:** the floor is divided into cells of 5 cm x 5 cm.
  - **White** = free. A sonar beam went through the cell.
  - **Black** = occupied. A sonar beam stopped in the cell.
  - **Gray** = unknown. No beam went into the cell.
- The sonar beam is approximately 15 degrees wide. Thus the free area has round edges.

**Expected results in the simulator:** most red dots are on the green lines. Areas more than 150 cm away are gray.

**Questions:**

- a. Measure the distance to one wall with a tape measure. Is the wall at the same distance on the map?
- b. Thin chair legs and walls at a large angle are not on the map. Why?
- c. The sonar beam is approximately 15 degrees wide. How does this change the shape of small objects on the map?
- d. Change the scan to 2-degree increments. Is the map better? How much more time is necessary for the scan?

---

## 6. Task 4: Move in a square, calculate the position and make a map

**Purpose:** Make a map when the position of the car is calculated. The car calculates its position from its steps. This is **dead reckoning**:

- Forward step: x = x + cmPerStep x cos(heading), and y = y + cmPerStep x sin(heading).
- Turn step: heading = heading + (turn angle that the camera measured).

The camera measures each turn: when the car turns, the corners in the photo move to the side. This is **visual odometry**. The car adds each sonar value to the map at the calculated position. If the calculated position has an error, the map has the same error.

**What the car does:** The car moves in a square with sides of 10 steps. At each corner, it makes a sonar scan. Then it turns 90 degrees to the left. During the turn, the camera measures the angle (feedback control).

**Procedure:**

1. Do task 2 first. Write its results in SETTINGS.
2. Put the car less than 1 m from walls or furniture. Make sure that the car has 1 m of clear floor in front of it and on its left side.
3. Put a piece of tape on the floor at the start position.
4. Set `TASK = 4`. Click **Run**.
5. At the end, measure the position of the car from the tape. Compare it with the calculated position in the Command Window.

### Figure 4.1: Square path and sonar map

![Task 4, Figure 4.1: Square path and sonar map](docs/images/task4_figure4_1.png)

**How to read Figure 4.1:**

- **Blue line:** the calculated path (dead reckoning). The green dot is the start.
- **Magenta dashed line:** the true path (simulator only). On the real car, use your tape measure.
- **White, black and gray cells:** the occupancy grid, as in Figure 3.1. All the sonar values are added at the calculated positions.
- If the blue line and the magenta line are not the same, the walls on the map are not at the correct positions.

**Expected results in the simulator:** the square closes. The calculated end position is approximately 10 cm from the start. The calculated path is approximately 10 cm or less from the true path.

**Questions:**

- a. Where does the car stop? Where does the car calculate that it stops?
- b. Find a wall that shows two times on the map. Which error causes this?
- c. Set `useCamera = false` in task 4. The car then counts the turn steps. Is the map better or worse?
- d. How can the car correct its position when it returns to the start? (Task 5 shows one method.)

---

## 7. Task 5: Make a camera map with visual SLAM

**Purpose:** Visual SLAM makes a map and finds the car in the map at the same time. MATLAB `monovslam` does these steps with one camera:

1. It matches corners in the photos (as in task 4).
2. It calculates the 3-D position of the corners. These are the **map points**.
3. It calculates the camera position at **key frames** (selected photos).
4. It identifies a place that it saw before. Then it corrects the map. This is **loop closure**.

One camera cannot measure size: 1 m and 1 cm look the same. Thus the map has no scale at first ("SLAM units"). At the end, the sonar measures the scale.

**What the car does:**

1. **Start the map:** 4 steps forward, then 4 steps back, again and again. The camera must see the room from two positions to measure depth. This can use 10 to 250 photos.
2. **Circle:** the car moves on a large circle to the left (approximately 1.25 turns).
3. **Scale:** the car moves straight to an object. The sonar distance decreases by the distance that the car moves. This gives the scale (cm for each SLAM unit).

If SLAM cannot find the position of the car (status `TrackingLost`), the car moves back along its path. If SLAM loses the position again on the circle, the car continues on the circle past that place.

**Procedure:**

1. Put the car in the middle of 2.5 m x 2.5 m of clear floor. The room must have furniture, posters or boxes. A blank wall or a floor that reflects light gives the camera no corners.
2. For the real car, calibrate the camera one time. Type `help calibrateElegooCamera` and do the procedure. Keep `elegooCameraIntrinsics.mat` in the kit folder.
3. Set `TASK = 5`. Click **Run**. Wait 5 to 8 minutes.

### Figure 5.1: Visual SLAM live view (changes during the run)

This figure changes after each photo. It shows the data that SLAM has at that time.

![Task 5, Figure 5.1: Visual SLAM live view](docs/images/task5_figure5_1.png)

**How to read Figure 5.1:**

- **(a) Camera photo:** the newest photo. The title shows:
  - the photo number,
  - the phase: `start` (start the map), `circle` or `scale`,
  - the SLAM status: `FrequentKeyFrames` or `TrackingSuccessful` (SLAM knows the position), or `TrackingLost` (SLAM cannot find the position),
  - the angle of the circle until now.
- **(b) SLAM top view:** what SLAM knows at this time, as seen from above.
  - **Gray dots:** map points (corners in 3-D), with floor and ceiling points included.
  - **Blue line:** the key frames, that is, the camera path.
  - The units are **SLAM units**, not cm. The scale is not known yet.

### Figure 5.2: Visual SLAM map (final)

SLAM makes this figure one time, at the end. It uses all the data after SLAM completes its corrections.

![Task 5, Figure 5.2: Visual SLAM map (final)](docs/images/task5_figure5_2.png)

**How to read Figure 5.2:**

- The line below the label shows the number of key frames and the scale.
- **Red dots:** map points 4 cm to 80 cm above the floor (objects). Floor and ceiling points are removed.
- **Blue line:** the SLAM path in cm. The green dot is the start of the map.
- **Black dashed line:** the true path (simulator only). **Green lines:** the true walls and furniture (simulator only).
- The map starts where SLAM made its first key frame. The x axis is the direction of the car at that time. Thus the room can look turned.
- If the blue circle is larger or smaller than the black circle, the scale has an error.

**Figure 5.1 and Figure 5.2 are different:**

| | Figure 5.1 (live view) | Figure 5.2 (final map) |
|---|---|---|
| When | After each photo, during the run | One time, at the end |
| Units | SLAM units (no scale) | cm (scale from the sonar) |
| Map points | All points | Only objects (4 cm to 80 cm above the floor) |
| True path and room | Not shown | Shown (simulator only) |
| Corrections | Only the corrections until now | All corrections, including loop closure |

**Expected results in the simulator:**

- The map starts after 220 to 250 photos.
- The map has 60 to 70 key frames.
- The SLAM path is 25 cm to 35 cm (RMS) from the true path.
- The scale error is between -15% and +20%. It changes each time.
- Sometimes SLAM cannot find the position, and the car stops early. If this occurs, do the task again.

**Questions:**

- a. Compare the SLAM path with the path of task 4. Which path returns nearer to the start?
- b. Where are there many map points? Where are there no points? Why?
- c. What occurs to the map if the scale has an error of 20%?
- d. Set `S.circle = [125 160 0.15]` (smaller circle) or `[145 160 0.15]` (larger circle). Which circle gives a better map?

---

## 8. Record your results

Copy this table. Write one row for the simulator and one row for the real car.

| | Simulator | Real car |
|---|---|---|
| Task 1: sonar value with no object (cm) | | |
| Task 1: number of corners in the photo | | |
| Task 2: S.cmPerStep (cm) | | |
| Task 2: S.degPerTurn (degrees) | | |
| Task 3: distance to one wall, map / tape measure (cm) | | |
| Task 4: calculated end position (x, y) (cm) | | |
| Task 4: true end position (x, y) (cm) | | |
| Task 5: number of key frames | | |
| Task 5: scale (cm for each SLAM unit) | | |
| Task 5: did the circle close? (yes / no) | | |

Save each figure: in the figure window, click **File > Save As**. Keep the label of the figure in the file name, for example `task3_figure3_1_realcar.png`.

---

## 9. Words that this guide uses

| Word | Meaning |
|---|---|
| Calibration | A measurement that finds the correct values for a sensor or a motion |
| Dead reckoning | Calculation of the position from the steps and turns of the car |
| Heading | The direction that the car points (degrees). 0 = the start direction. Positive = to the left |
| Key frame | A photo that SLAM selects and keeps in the map |
| Localization | The robot calculates its position and its heading |
| Loop closure | SLAM identifies a place that it saw before. Then it corrects the map |
| Map point | A corner that SLAM calculated in 3-D |
| Occupancy grid | A map that divides the floor into cells: free, occupied or unknown |
| ORB corner (feature) | A small area of a photo that the computer can find again in a different photo |
| Pan servo | The motor that turns the sensor head to the left and right |
| Pose | Position (x, y) and heading together |
| PWM | The motor power: -255 (full reverse) to 255 (full forward) |
| RMS | Root mean square. The average size of an error |
| Scale | The number of cm for each SLAM unit |
| SLAM | Simultaneous Localization And Mapping |
| SLAM unit | The length unit of a map from one camera. It is not a known length |
| Sonar | The ultrasonic sensor. It measures the distance to the nearest object |
| Visual odometry | Calculation of the motion of the camera from the corners in the photos |
