# Feature based SLAM with Fourier Series

This is the project for **Winter School**.

Simultaneous localization and mapping (SLAM) is the computational problem of
constructing a map of an unknown environment while simultaneously tracking a robot's location within it. Feature based SLAM is one technique to extract features from sensors to solve the problem. Most of the early work for feature based SLAM approaches considers only point features, while in this project you will learn how
to use Fourier series to represent features.

![Description](D:\Dropbox\1.Documents\2.UTS\WinterSchool\WinterSchoolProj\FOURIER-SERIES-SLAM-FOR-WINTER-SCHOOL\readmeIMG.png)

---

*Supervisor*: 

* Dr. Jiaheng Zhao, PhD in University of Technology Sydney and Beijing Institute of Technology, jiaheng.zhao@student.uts.edu.au;

* Mr. Tiancheng Li, PhD student at Centre for Autonomous Systems, University of Technology Sydney, tiancheng.li-1@student.uts.edu.au;

## Purpose

In this project, you will learn basic knowledge of feature based SLAM and how to use Fourier series to parameterize features. Also you can follow the basic code 
ow based on Matlab.

## Project step

The participants will be asked to complete the following exploration steps based on the provided code, including:

#### 1. Download template code

#### 2. Add to searching path

Execute `startup.m` to add path before running.

#### 3. Task 1 - Fit features with given example data

* Open `fsFitting.m`.
* Read code and complete function `DataProcessing/fitWithFS.m` at *Line 29*.

#### 4. Task 2 - Experiment with the simulated dataset

* Open `fsSLAM.m`.

* Use the dataset `demo_simu` (*Line 12*):

  ```matlab
  experiment = 'demo_simu';  
  ```

* The format of `Xstate` is defined as follows:

  ```matlab
  				    		Coloumn
  		______________________________________________________________
  			1                    2                     3
  Xstate = [ value, pose->1 feature->2, id_this]
           
  ```

  * The 1st column is the value. 
  * The 2nd column denotes the type of variables. Pose part (arrayed by [x; y; theta]) is noted by 1, while feature part (arrayed by [center_x; center_y; a0-an; b1-bn]) is noted by 2.
  * The 3rd column labelled the index of this item.

  ---

  The format of `Zstate.center`  and `Zstate.odom` are defined as follows:

  ```matlab
  								Coloumn
  		______________________________________________________________
               1           2               3            4
  Zstate = [ value, pose->1 feature->2, id_this, id_relativeto]
  ```

  * The 1st column is the value. 
  * The 2nd column denotes the type of variables. Pose part (arrayed by [x; y; theta]) is noted by 1, while feature part (arrayed by [center_x; center_y; a0-an; b1-bn]) is noted by 2.
  * The 3rd column labelled the index of this item.
  * The 4th column is the reference pose index.

  ---

  The format of `Zstate.fs` is defined by a cell structure: (m steps) x (n features), each cell contains the observed points.

  ---

  The format of `feaOccurredID` is defined as follows:

  ```matlab
  feaOccurredID with format: [newID   preID  occuredStepID]
  % Example: 
  % If the first step observes feature 1 3 4, then feaOccurredID is
  %				1   1   1
  %      	        2   3   1
  %               3   4   1
  % The second step see feature 1 4 5, feaOccurredID is
  %               1   1   1
  %               2   3   1
  %               3   4   1
  %               4   5   2
  ```

  

* Read code and complete cost function `ToolSolver/FuncfFS.m` after *Line 82*.

  

* Read code and complete Jacobian matrix function `ToolSolver/FunJacFS.m`

  Use the given code, complete *Line 205*.

* Run `fsSLAM.m`

* Adjust `lidar.fsN_local` in the sub-function ` setLidarParameters1()` to see what will happen.

#### 5. Task 3 - Experiment with the Techlab dataset

* Use the dataset `demo_techlab` (*Line 12*):

  ```matlab
  experiment = 'demo_techlab';  
  ```

* Run `fsSLAM.m` based on the last task.
* Adjust `lidar.fsN_local` in the sub-function ` setLidarParameters2()` to see what will happen.

---

###### Bonus time!

What if you don't know the data association?

Try to neglect `scan.scan(1,:) ` and use odometry to associate features!

*Hint: You can project points back to the initial frame using odometry, and then cluster nearest neighbour*.

#### 6. *Task 4 - Experiment with the car park dataset

This task is not indispensable. In case you've finished all the tasks above, try to manipulate a more general data.

* Use the dataset `demo_carpark` (*Line 12*):

  ```matlab
  experiment = 'demo_carpark';  
  ```

* Run `fsSLAM.m` based on the last task.
* Adjust `lidar.fsN_local`, `lidar.fsN_rect`, and `lidar.fsN_border` for different types of features in the sub-function ` setLidarParameters3()` to see what will happen.

#### 7. Remove searching path

Execute `clearfolder.m` to remove path.



## Reference

[1] Zhao, J., Li, T., Yang, T., Zhao, L., & Huang, S. (2021). 2D Laser SLAM With Closed Shape Features: Fourier Series Parameterization and Submap Joining. IEEE Robotics and Automation Letters, 6(2), 1527-1534