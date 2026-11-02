ELEGOO SMART ROBOT CAR V4: MAP AND POSITION KIT
ROBT 1010 Intro to Robotics Engineering
(This text uses ASD-STE100 Simplified Technical English.)

=========================================================================
1. START HERE
=========================================================================
Open STUDENT_GUIDE.html in a web browser (double-click it). The guide
shows each step, each figure and how to read it, the expected results and
what to do if a problem occurs. This file is a short summary.

=========================================================================
2. PURPOSE
=========================================================================
This kit shows the two basic functions of a mobile robot:
  - Localization: the robot calculates its position and its heading.
  - Mapping: the robot makes a map of the objects near it.
The car uses its sensors for these functions: the sonar (ultrasonic
sensor), the camera and the gyro. It does not use a GPS.

=========================================================================
3. FILES IN THIS FOLDER
=========================================================================
  STUDENT_GUIDE.html        The guide (open it in a web browser).
  STUDENT_GUIDE.md          The same guide as text (for GitHub).
  docs/images/              The example figures of the guide.
  elegooMappingTemplate.m   The template. All five tasks are in this file.
                            This is the only file that you change.
  ElegooRobotCar.m          Commands for the real car (Wi-Fi connection).
  ElegooSimCar.m            The simulator: a car in a living room in MATLAB.
  simAssets/                Photos of the surfaces in the simulated room.
  calibrateElegooCamera.m   Camera calibration for task 5 on the real car.
  README.txt                This text.

Do not move the files out of this folder. Do not change the names of the
files.

=========================================================================
4. THE TASKS
=========================================================================
  1  Read the sensors.
  2  Calibrate the motion: measure one forward step and one turn step.
  3  Make a sonar map from one position.
  4  Move the car in a square. Calculate the position and make a map.
  5  Make a camera map with visual SLAM.
Do the tasks in sequence. Task 4 uses the results of task 2.

=========================================================================
5. SHORT PROCEDURE
=========================================================================
  1. Unzip the kit. Windows: right-click the zip file > Extract All >
     Extract. Mac: double-click the zip file in Finder.
  2. Open MATLAB. Set the current folder to the folder that contains
     elegooMappingTemplate.m.
  3. In the Current Folder panel of MATLAB, double-click
     elegooMappingTemplate.m.
  4. Set TASK = 1. Keep USE_SIMULATOR = true.
  5. On the Editor tab, click Run. If MATLAB shows "Change Folder" and
     "Add to Path", click "Change Folder".
  6. Do the same for TASK = 2, 3, 4 and 5 in the simulator.
  7. Then do each task on the real car. Connect the computer to the Wi-Fi
     network of YOUR car (ask your TA for the name). Set
     USE_SIMULATOR = false. Click Run.

=========================================================================
6. SAFETY
=========================================================================
CAUTION: OPERATE THE CAR ON THE FLOOR ONLY. DO NOT OPERATE IT ON A TABLE.
         THE CAR CAN FALL AND BREAK.
CAUTION: KEEP A CLEAR FLOOR AREA OF 1 m x 1 m (2.5 m x 2.5 m FOR TASK 5).
         IF THE FLOOR IS NOT CLEAR, THE CAR CAN HIT AN OBJECT.
CAUTION: THE CAR MOVES BACK WITHOUT A SENSOR. KEEP THE FLOOR BEHIND THE
         CAR CLEAR.
CAUTION: TO STOP THE CAR, PUSH CTRL+C IN THE MATLAB COMMAND WINDOW (ALSO
         ON A MAC). IF THE CAR DOES NOT STOP, LIFT THE CAR. SET THE POWER
         SWITCH TO OFF.
