# 6D Object Pose Estimation using RGBD Data and Fast-ICP

## Problem Statement
Given RGBD Images and a model of the object, segment, reconstruct and estimate 6DOF pose of the object.

## Usage Guide:
1. Run `WrapperCar.m` for running Test Script for Car Planner.
2. Change the flags in Wrapper.m as needed.
3. By default, the code runs uses Liquid Container dataset (change folder name if needed).
4. By default, the code uses sub-sampling, truncation and point to plane method, does not display all iteration and save video.

## Report:
You can find the report [here](Report/ESE650Project6.pdf).

## Sample Outputs:
<a href="http://www.youtube.com/watch?feature=player_embedded&v=41MlKuzzLKk
" target="_blank"><img src="http://img.youtube.com/vi/41MlKuzzLKk/0.jpg" 
alt="Rotplot Video" width="240" height="180" border="10" /></a>

## References:
1. Arun, K. Somani, Thomas S. Huang, and Steven D. Blostein, Leastsquares
fitting of two 3-D point sets, IEEE Transactions on Pattern
Analysis and Machine Intelligence, Vol. 5, pp. 698–700, 1987.
2. Low, Kok-Lim, Linear least-squares optimization for point-to-plane
icp surface registration, University of North Carolina Chapel Hill,
Vol. 4, 2004.
3. Rusinkiewicz, Szymon, and Marc Levoy, Efficient variants of the ICP
algorithm, Proceedings of 3-D Digital Imaging and Modeling, 2001.

