# Writeup: Track 3D-Objects Over Time

### 1. Write a short recap of the four tracking steps and what you implemented there (filter, track management, association, camera fusion). Which results did you achieve? Which part of the project was most difficult for you to complete, and why?

In the first step, I implemented an Extended Kalman Filter for tracking a single real-world target using lidar measurement input. The EKF included the prediction step to estimate the object's future position and velocity, and the update step, where actual measurements adjusted these predictions. Achieving a low Root Mean Square Error (RMSE) indicated a successful implementation.

In the second step, I developed a system to manage multiple tracks, including initializing new tracks based on unassigned measurements, updating track scores, and deleting tracks that were no longer relevant. This step was crucial for multi-object tracking, ensuring that the system could dynamically handle objects entering and leaving the scene.

In the third step, I implemented a single nearest neighbor data association mechanism, using the Mahalanobis distance to associate measurements to tracks. This process was vital to ensure that each measurement updated the correct track.

In the fourth and final step, I integrated camera data with lidar data to enhance the tracking system. This involved transforming vehicle coordinates to camera coordinates and projecting these onto image space, adding a new dimension to the tracking capabilities.

The most challenging part was the implementation of the EKF, particularly ensuring the accuracy of the state transition and measurement models. It required a solid understanding of both the theoretical aspects of Kalman filtering and practical aspects of coding these models.

### 2. Do you see any benefits in camera-lidar fusion compared to lidar-only tracking (in theory and in your concrete results)? 
In theory, camera-lidar fusion offers several advantages over lidar-only tracking such as Improved Object Classification, Greater Field of View, and Redundancy and Robustness. In my project results, I observed that the fusion of camera data provided better context and improved the accuracy of object tracking, particularly in complex scenarios where lidar data alone was insufficient.

### 3. Which challenges will a sensor fusion system face in real-life scenarios? Did you see any of these challenges in the project?
In real-life scenarios, sensor fusion systems face several challenges such as Sensor Calibration and Alignment, Varied Sensor Limitations, and Data Processing Overhead. During the project, aligning and calibrating camera data with lidar data was a glimpse into these real-world challenges, particularly ensuring that the data from both sensors corresponded to the same physical objects in space.

### 4. Can you think of ways to improve your tracking results in the future?
To improve tracking results in the future, I would taking steps into areas such as Advanced Sensor Fusion Techniques, Dynamic Model Adaptation, and Real-Time Processing Optimization. Especially the optimization as optimizing algorithms for real-time processing is critical for practical deployment in autonomous vehicles and other similar applications.


