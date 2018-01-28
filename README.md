# Extended Kalman Filter Project

The goal of this project is to implement an Extended Kalaman Filter using Radar and Lidar measurements.

## Project Structure

The crux of the code is implemented in 3 files:

* FusionEKF.cpp
* kalman_filter.cpp
* Tools.cpp

FusionEKF: This is the module that is responsible for initializing and calling the kalaman filter prediction and update functions 
based on the type of the input measurement. The inpupts can be either from a Radar or from a Lidar

kalaman_filter.cpp: This is the module that implements the kalman filtering prediction and update equations - both the linear case(Lidar) 
and the non linear case (Radar)

Tools.cpp: This module implements the RMS calculation as well as the Jacobian matrix calculation that is required for the non-linear update
equations

## Results

For the dataset under Data folder, the RMSE values when using both Radar and Lidar inputs are given by:


|   Variable         	|   RMSE 					| 
|:---------------------:|:-------------------------:| 
|   px        	 		| 0.0973178 		    	| 
|   py        	 		| 0.0854597 		    	| 
|   vx        	 		| 0.451267  		    	| 
|   vy        	 		| 0.439935  		    	| 


## Experiment with using only one of the sensors


In this experiment, only one of the sensor measurements was used as input to the Kalman Filter.  The RMSE results are given below.

Usng only Lidar measurents:


|	Variable         	|     RMSE 					| 
|:---------------------:|:-------------------------:| 
| px        	 		| 0.183795   		    	| 
| py        	 		| 0.154202  		    	| 
| vx        	 		| 0.605092  		    	| 
| vy        	 		| 0.485836  		    	| 


Usng only Radar measurents:


|	Variable         	|     RMSE 					| 
|:---------------------:|:-------------------------:| 
| px        	 		| 0.233172   		    	| 
| py        	 		| 0.335998  		    	| 
| vx        	 		| 0.617771  		    	| 
| vy        	 		| 0.678604  		    	| 



We can see that the Radar by itself gives the least accurate estimates. Using just Lidar gives better accuracy in the final measurements.
However, fusing information from both the sensors gives the best results.

