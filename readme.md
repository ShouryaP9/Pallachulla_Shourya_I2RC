# Preseason Digital Notebook Example
Name: Shourya Pallachulla

Section: I2RC

Week: 3


## Code

The main topic this week was: PID

Commands: **[PID.Java]**

Subsystems: **[Drivetrain]**

### How does the code work?
The code uses the calculate method of PID Controller using a setpoint angle that you're trying to turn to, and the current angle of the robot that it gets with the getCurrentAngle() function in Drivetrain.
When it initializes, it resets the gyro and sets the speed of the robot to 0. While executing, it uses the calculate method to calculate how much the robot still has to turn to reach the setpoint angle.
Once it reaches the setpoint angle, the speed is reset to 0, which stops it from turning more.


### Important notes for future reference
Notes about git, and helpful resources, etc. 

Please put them here, they will really help you in the future 
