# Known issues and open work

Status on 2026-11-11. This file is for the course maintainers. The student guide is [vslam_examples/README.md](vslam_examples/README.md).

## Fixed on 2026-11-11

- **The map never started** (`visualSlamDemo` and template Task 5). `monovslam` starts a map only on a photo with at least `MaxNumPoints` (1000) corners. The ESP32 photos and the simulator photos have about 650. Now SLAM gets a photo with more contrast (`adapthisteq`), which has 1300 to 1800 corners. A new SLAM object starts after 8 photos (one sweep), not after 4.
- **Striped photos at 640 x 480.** `PhotoSize` is now 8 (400 x 296).
- **Striped lower part of photos at 400 x 296** (48 of 81 photos in run `vslamRun_20261111_144818`). `visualSlamDemo` takes photos until two in a row agree. The cause is not known. Possible causes: the 24 MHz camera clock in the firmware (Espressif examples use 20 MHz), a low battery, or heat. `steps.csv` now records the battery voltage at each step.
- **No log of real-car runs.** `visualSlamDemo` now writes `debug.txt` and `steps.csv` in each `vslamRun_...` folder.

## Open issues in the student template (`elegooMappingTemplate.m`)

1. **Task 5 stops early in the simulator.** In one test after the fix, the map started, but SLAM lost its position 7 times on the circle. Before the fix, about 1 of 3 runs stopped early. The cause is not known. Possible cause: the circle steps of the template are larger than the circle steps of `visualSlamDemo` (about 4 degrees for each step). The full simulator run of `visualSlamDemo` did not get lost.
   - To do: run Task 5 five times in the simulator. Compare the circle settings with `visualSlamDemo`.
2. **The simulator results in the README are old.** The README says that the Task 5 map starts after 220 to 250 photos. That was the bug. Measure again: the map start, the key frames, the path error and the number of complete runs.
3. **The R2026a note in the README is possibly wrong.** The README says that the simulated camera shows mostly floor on R2026a, and that Task 2 stops with "too few corners". On R2026a Update 5, the simulator photos showed the furniture. Task 2 was not tested.
   - To do: run Task 2 on R2026a. Also try `adapthisteq` in Task 2 and Task 4, because they also count corners.
4. **The template does not have the real-car protections of `visualSlamDemo`.** It does not do the two-photo stripe check, and it does not write `debug.txt` or `steps.csv`. On the real car, it can give striped photos to SLAM, and it keeps no log.

## Open documentation work

- Write the procedures in ASD-STE100 in the READMEs: stop early and build the map (STOP, wait for "Auto run ended", then **Finish & Build Map**), the steps before a run (switch the car off and on, close the `/stream` page), and what to do with `debug.txt` after a test.
- Make the architecture diagrams simpler, mainly in [vslam_examples/README.md](vslam_examples/README.md). The Task 5 diagram still says "new SLAM after 4 photos".

## Not tested on the real car yet

All of the 2026-11-11 changes were tested only on saved photos and in the simulator. Before the next real-car test: charge the battery, switch the car off and on, close the `/stream` page, then run `clear classes` and `visualSlamDemo()`.
