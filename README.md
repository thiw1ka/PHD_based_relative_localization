# GM PHD Filter with Variable PD PS Estimation

In this project we explored automated estimation of PD (Probability of detection) and PS (Probability of survival) of PHD (Probability hypothesis density) filter.
PHD filter is a multi-target tarcking filter that estimate the quantity of the targets and it's states without prior information.
for more information please refer following paper for standard PHD filter. [The Gaussian Mixture Probability Hypothesis Density Filter](https://ieeexplore.ieee.org/abstract/document/1710358?casa_token=teDbNPXJ6J8AAAAA:HANXPhV4vcg53tWRbYMyOfVzhvFKYyWyCeB3UDq01AcG-DWYJR197mbbsGwhl7wdzC_PHOmruA).
In this extension of PHD filter, we calculate PD and PS online for each estimate for every time step. The advantage of such system would be benificial when observing targets are consist with non similar probability of detections. The publication will be added here in future update. 

## Problem setting
In this project the filter is implemented as a relative localization module that runs by a single project. The localization module will estimate other robots location in robot frame using sensor measurements and ego-motion of the robot. The PHD based relative localization was introduced in our paper - [A PHD Filter Based Localization System for Robotic Swarms](https://par.nsf.gov/servlets/purl/10332874).

### Gazebo simulation ROS setup
We simulated 5 robots inside a squared testing area equipped with camera and lidar sensors. Robot 1 will execute the filter and estimate each robot position in each time step.

Lidar Measurements
lidar provides 720 points covering 360 degrees field of view and it is attache on XY robot origin. An seperate algorithem will analyse each lidar scan and extracts point set that represnt a robot. Centroid of this blob is provided to the Filter.

Camera Measurerments.
Camera has 60 degrees field of view. Each camera frame is provided to a color extracting algorithem to extract blue color lidar structure. This algorithem provides the direction of the robot in camera frame of reference. We provided this detail as a measurement to the filter.

Estimates - filter results
Filter results are represnted in world frame of view for easy understanding. published in filter results topic.
Rviz has a pre-setup topics that can be used to see the filter results.

## Sample videos of the project.

**This video shows both gazebo simulation and estimates in the rviz visulizer.**

https://github.com/thiw1ka/PHD_Filter_Variable_PD_PS_Estimation/assets/45106731/4e463888-fa58-4a1c-b90c-07fd56c9001e

**This video shows filter results.**
legend -
  - Red color dots - lidar points
  - yellow color box filter estimation
  - stright line - camera measurements projected to world frame



https://github.com/thiw1ka/PHD_Filter_Variable_PD_PS_Estimation/assets/45106731/5a61dece-50ab-4165-9376-196f0a045f42

## Project Files
All project files are ros based packages and includes following sub packages.
-PHD filter extension
-Robot gazebo simulation
-Lidar robot extraction
-Utility function

