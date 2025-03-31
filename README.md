# MAIN README

Autonomous Flight of Micro Air Vehicles: Group 11
=============

This project focuses on autonomous indoor navigation for micro-sized drones, specifically the Parrot Bebop drone, in an obstacle-rich environment(Cyber Zoo Lab). The goal is to enable safe and efficient navigation using vision-based perception and optimized control strategies, running entirely onboard the drone without external processing.

Project Pipeline Overview
-----------------
Our approach involves:
- **Vision-based navigation** using a single forward-facing camera for real-time environment perception.
- **Collision avoidance** through a combination of green pixel segmentation and edge detection.
- **State-based decision-making** to dynamically adjust flight behavior based on perceived safety levels.


Compilation and demo simulation
-------------------------------

1. Clone the repository and install 

```
git clone https://github.com/FabianDumitrascu/MAVgroup11.git
cd MAVgroup11/
./install.sh
```

2. Once installation is done, open the paparazzi center.
```
cd MAVgroup11/
./paparazzi
```

3. **Select Aircraft**:  
   - In the top-left dropdown, choose **`bebop orange avoid`**. 
 
4. **Set Build Target**:  
   - Under **`Build`**, select **`nps`** in the **`Target`** dropdown.  
5. **Compile the Code**:  
   - Click **`Clean`**, then **`Build`**.  
6. **Launch the Simulation**:  
   - Go to the **`Operation`** tab.  
   - Under **`Control Panel`**, select **`userconf/tudelft/course control panel.xml`**.  
   - Under **`Session`**, choose **`Simulation - Gazebo`** and click **`Start Session`**.  
7. **Adjust Parameters (Optional)**:  
   - During simulation, color segmentation and edge detection parameters can be fine-tuned to improve performance  in the Paparazzi Center.  
8. **Stop the Simulation**:  
   - Click **`Stop All`** in the Paparazzi Center when done.  

During simulation, the parameters for color segmentation and edge detection can be modified for better perfection during the run


Uploading the embedded software
----------------------------------

1. Power the flight controller board while it is connected to the PC with the USB cable.

2. From the Paparazzi center, select the "ap" target, and click "Upload".


Flight
------

1.  From the Paparazzi Center, select the flight session and ... do the same as in simulation !
