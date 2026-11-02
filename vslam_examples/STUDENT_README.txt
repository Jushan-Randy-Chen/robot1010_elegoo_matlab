ELEGOO SMART ROBOT CAR V4: MAP AND POSITION KIT
ROBT 1010 Intro to Robotics Engineering
(This text uses ASD-STE100 Simplified Technical English.)

=========================================================================
1. PURPOSE
=========================================================================
This kit shows the two basic functions of a mobile robot:
  - Localization: the robot calculates its position and its heading.
  - Mapping: the robot makes a map of the objects near it.
The car uses its sensors for these functions. It does not use a GPS.
You do five tasks. Each task adds one new idea to the task before it.

=========================================================================
2. FILES IN THIS FOLDER
=========================================================================
  elegooMappingTemplate.m   The template. All five tasks are in this file.
                            This is the only file that you change.
  ElegooRobotCar.m          Commands for the real car (Wi-Fi connection).
  ElegooSimCar.m            The simulator: a car in a living room in MATLAB.
  simAssets\                Photos of the surfaces in the simulated room.
  calibrateElegooCamera.m   Camera calibration for task 5 on the real car.
  README.txt                This text.
  STUDENT_GUIDE.md          The full guide: each task, each figure and how to
                            read it, the expected results and a results table.
  docs\images\               Example figures for the guide (from the simulator).

Do not move the files out of this folder. Do not change the names of the
files. The template must find the other files in the same folder.

=========================================================================
3. WHAT YOU MUST HAVE
=========================================================================
  - MATLAB R2024a or later (tested with R2026b), with these toolboxes:
      Computer Vision Toolbox
      Image Processing Toolbox
  - For the real car: an ELEGOO Smart Robot Car V4 with the course
    firmware, a charged battery, and a computer with Wi-Fi.
  - For task 2 and task 3 on the real car: a tape measure.
  - For task 5 on the real car: a printed checkerboard (see section 7).

=========================================================================
4. SENSORS
=========================================================================
  Ultrasonic sensor (sonar)  Measures the distance to the nearest object
                             (2 cm to 150 cm). It is on the pan servo head.
  Camera                     Takes photos. The computer finds corners in
                             the photos. It is on the pan servo head.
  Gyro                       Measures the turn rate of the car.
  Pan servo                  Turns the sensor head to the left and right.
  Motors (4)                 Move the car. The PWM value sets the power.

=========================================================================
5. THE TASKS
=========================================================================
TASK 1: READ THE SENSORS
  Sensors: sonar, camera, gyro.     Time: approximately 1 minute.
  The car stays at one position. You move your hand in front of the sonar.
  The car takes one photo and turns for 0.8 seconds.
  This task shows: the range and the noise of each sensor.

TASK 2: CALIBRATE THE MOTION
  Sensors: sonar, camera.           Time: approximately 1 minute.
  The car moves 8 steps forward and 8 steps back. The sonar measures the
  distance of one step. Then the car turns approximately 400 degrees. The
  camera measures the angle of one turn step.
  Result: two values (S.cmPerStep and S.degPerTurn). Write them in SETTINGS.
  This task shows: calibration. A robot must measure its own motion.

TASK 3: MAKE A SONAR MAP FROM ONE POSITION
  Sensors: sonar, pan servo.        Time: approximately 15 seconds.
  The car does not move. The servo turns the sonar from right to left.
  The sonar measures the distance at each 5 degrees.
  Result: scan points and an occupancy grid (free, occupied, unknown).
  This task shows: mapping when the position of the robot is known.

TASK 4: MOVE IN A SQUARE, CALCULATE THE POSITION AND MAKE A MAP
  Sensors: sonar, camera, pan servo.  Time: approximately 1 minute.
  The car moves in a square. It counts its steps and the camera measures
  its turns (dead reckoning). The car adds the sonar values to the map at
  the calculated position.
  Result: the calculated path and a sonar map.
  This task shows: small errors in the position add up. The map becomes less
  accurate.

TASK 5: MAKE A CAMERA MAP WITH VISUAL SLAM
  Sensors: camera, sonar.           Time: approximately 3 to 8 minutes.
  The car moves on a large circle. Visual SLAM makes a 3-D map of corners
  and finds the car in the map at the same time. When the car returns to
  a known place, SLAM corrects the map (loop closure). At the end, the
  sonar measures the scale of the map.
  Result: the SLAM path and a map of the room.
  This task shows: SLAM corrects the errors that task 4 cannot correct.

HOW THE TASKS ARE DIFFERENT
  Task  Car moves?        Position from          Map from
  ----  ----------------  ---------------------  --------------------------
  1     Turns one time    Not calculated         No map
  2     Line, then turns  Sonar and camera       No map
  3     No                Known (does not move)  Sonar
  4     Square            Steps and camera       Sonar
  5     Circle            Visual SLAM            Camera (and sonar scale)

=========================================================================
6. PROCEDURE: SIMULATOR FIRST, THEN THE REAL CAR
=========================================================================
  1. Unzip the folder. Open MATLAB.
  2. Set the MATLAB current folder to this folder.
  3. Open elegooMappingTemplate.m.
  4. Set TASK = 1 and USE_SIMULATOR = true.
  5. Click Run (or push F5). Look at the results.
  6. Do step 4 and step 5 for TASK = 2, 3, 4 and 5.
     In the simulator, a green outline shows the true room. A dashed
     line shows the true path. Use them to examine your results.
  7. Then do the tasks on the real car:
     a. Set the power switch of the car to ON. Connect the computer to the
        Wi-Fi network of the car ("ELEGOO-...").
     b. Set USE_SIMULATOR = false.
     c. Do TASK = 1, then TASK = 2.
     d. Write the results of task 2 in SETTINGS (S.cmPerStep and
        S.degPerTurn). The results of the simulator are not correct for
        the real car.
     e. Do TASK = 3, 4 and 5.
  8. Compare the results of the simulator and the real car. Answer the
     questions in each task section of the template.
  9. Read STUDENT_GUIDE.md. It shows each figure and tells you how to read
     it. Each figure has a label, for example "Task 3, Figure 3.1".

CAUTION: KEEP THE CAR AWAY FROM STAIRS AND TABLE EDGES.
         THE CAR CAN FALL AND BREAK.
CAUTION: KEEP 1 m OF CLEAR FLOOR AROUND THE CAR (2.5 m FOR TASK 5).
         IF THE FLOOR IS NOT CLEAR, THE CAR CAN HIT AN OBJECT.
CAUTION: TO STOP THE CAR, PUSH CTRL+C IN THE MATLAB COMMAND WINDOW.
         IF MATLAB STOPS, THE MOTORS STOP AFTER 1 SECOND.

=========================================================================
7. TASK 5 ON THE REAL CAR: CALIBRATE THE CAMERA
=========================================================================
Do this procedure one time for each car, before task 5 on the real car.
  1. Type: calibrateElegooCamera("print")
  2. Print checkerboard_print.png at 100% scale. Attach it to a flat board.
  3. Measure one square with a ruler (mm).
  4. Type: calibrateElegooCamera(25)   (use your square size in mm)
  5. Do the steps that show in the help text: type
     help calibrateElegooCamera
  6. Keep elegooCameraIntrinsics.mat in this folder.
If you do not calibrate, task 5 uses an approximate camera. The map is
then less accurate.

=========================================================================
8. IF A PROBLEM OCCURS
=========================================================================
  Problem                          Action
  -------------------------------  ----------------------------------------
  "Undefined function" error       Set the MATLAB current folder to this
                                   folder.
  The real car does not connect    Make sure that the computer is on the
                                   Wi-Fi network of the car. Set the power
                                   switch to OFF, then to ON. Do the task
                                   again.
  The sonar shows NaN or 0         Examine the cable of the sonar.
  The photo is upside down         Set S.flipImage = true in SETTINGS.
  In task 3, pan(150) turns the    Set S.panLeftIsBigger = false in SETTINGS.
  sensor head to the right
  Task 4 turns more or less        Do task 2 again. Write the new values in
  than 90 degrees                  SETTINGS.
  Task 5: "SLAM did not make a     Put boxes, posters or books in the room.
  map"                             Calibrate the camera. Do the task again.
  Task 5: the car stops early      The room has too few corners. Move the
  (TrackingLost)                   car to a different area.
  The car hits an object           Push CTRL+C. Remove objects from the
                                   floor.
