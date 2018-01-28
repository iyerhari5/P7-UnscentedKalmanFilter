# Unscented Kalman Filter Project

The goal of this project is to implement an Unscented Kalaman Filter using Radar and Lidar measurements.

[//]: # (Image References)

[image1]: ./Images/NIS-Lidar.png
[image2]: ./Images/NIS-Radar.png


## Project Structure

The crux of the code is implemented in 3 files:

* ukf.cpp
* Tools.cpp

ukf.cpp: This is the module that implements the unscented kalman filtering prediction and update equations for both the Lidar and Radar cases.

Tools.cpp: This module implements the RMS calculation

## Results

For the dataset under Data folder, the RMSE values when using both Radar and Lidar inputs are given by:

|   Variable         	|   RMSE 					| 
|:---------------------:|:-------------------------:| 
|   px        	 		| 0.0682    		    	| 
|   py        	 		| 0.0828     		    	| 
|   vx        	 		| 0.3365    		    	| 
|   vy        	 		| 0.2184    		    	| 



## Experiment with using only one of the sensors


In this experiment, only one of the sensor measurements was used as input to the Kalman Filter.  The RMSE results are given below.


Usng only Lidar measurents:


|	Variable         	|     RMSE 					| 
|:---------------------:|:-------------------------:| 
| px        	 		| 0.1709     		    	| 
| py        	 		| 0.1487    		    	| 
| vx        	 		| 0.6200    		    	| 
| vy        	 		| 0.2707    		    	| 


Usng only Radar measurents:


|	Variable         	|     RMSE 					| 
|:---------------------:|:-------------------------:| 
| px        	 		| 0.2137    		    	| 
| py        	 		| 0.2920    		    	| 
| vx        	 		| 0.3840    		    	| 
| vy        	 		| 0.2306    		    	| 



We can see that the Radar by itself gives worse estimates of the position. However vx and vy are better 
estimated from the Radar only measurements compared to the Lidar. However, fusing information from both the sensors gives the best results.


## Comparison to Extended Kalman Filter Results

For the dataset under Data folder, the RMSE values when using both Radar and Lidar inputs for the Extended Kalman Filter are given by:

|   Variable         	|   RMSE 					| 
|:---------------------:|:-------------------------:| 
|   px        	 		| 0.0973178 		    	| 
|   py        	 		| 0.0854597 		    	| 
|   vx        	 		| 0.451267  		    	| 
|   vy        	 		| 0.439935  		    	| 

We see that the Unscented Kalman Filter is giving much better performance for estimating all the 4 parameters.


## Consistency Checks of the Uncented Kalman Filter

Figure below plots the NIS values for only the Lidar measurements. Since the measurement space is in 2D, the Chi-squared limit for 95% is 5.991
as shown with the red line. We see that the filter is consistent for the Lidar measurements.

![alt text][image1]


Figure below plots the NIS values for only the Radar measurements. Since the measurement space is in 3D, the Chi-squared limit for 95% is 7.815
as shown with the red line. We see that the filter is consistent for the Lidar measurements.

![alt text][image2]
