# README #

This folder contains EE586 project of team 2 - KineTeris.

### Weekly progress ###

#### Week 7: ####
* Coding part: Made template codes working and figured out the data structure of input video/images; created simple test cases to manipulate the output, for example, displaying an image of tetris user menu. 
* Research part: read papers about gesture and hand tracking. Found open-source Matlab codes to run; achieved generating binary image output through PC camera, without locating hands.
* Game part: Got Matlab codes of Tetris game with a very simple AI. Trying to design a new AI and figure out the core functions of the game. 


#### Week 8: ####
Goal: Implement RGB2YCrCb, YCrCb2RGB, and displaying of 720*480 images on the computer screen and projector. Add 2nd depth search in AI, and random block drop logic. 
Extract the finger tips by denoting locally convex boundaries, based on the Matlab platform.

#### Week 10: ####
Week 8 goals completed.
Goal: Output two tetris fields on the computer monitor with some blocks (game prototype) generated on the DSP Board. Convert some game engine from MATLAB to C. Look into TI video libraries.
Finish MATLAB image processing: segmentation (foreground extraction), binarization, fingertip detection (convex hull or k curvature), angle calculation, and tracking with Kalman filter.

#### Week 11: ####
Week 10 goals partially completed, i.e. implemented the Otsu's method on segmentation and binarization. In addition, TI libraries and OpenCV are not considered. 
Goal: 
Implement morphological operations like erosion and dilation for de-noising. Initialize the seven tetris block shapes. 

#### Week 12: ####
Week 11 goals were not completed fully. Some issues like delay and unknown memory leaking occurred when implementation.
Goal: 
Fix bugs on image processing part. For game engine, try to implement block dropping.

#### Week 13: ####
Week 12 goals were completed.
Goal: 
Implement contour tracing algorithm. For game engine, try to implement rotation and simple AI (first step consideration).

#### Week 14: ####
Contour tracing was achieved but with some bugs.
Goal: 
Fix that the contour tracing function can cause program "freezing". Based on the resulting contour, try to find the fingertips and valleys via k-curvature algorithm. Implement depth-2 search AI.

#### Week 15: ####
Week 14 goals were completed.
Goal: 
Calculating the shifts distance and rotation angle, then translate them into game commands. Calibrate depth-2 AI penalty weights and some small block collision bugs. 