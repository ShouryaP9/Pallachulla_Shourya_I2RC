// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Drivetrain extends SubsystemBase {
  private final WPI_TalonSRX leftDriveTalon;
  private final WPI_TalonSRX rightDriveTalon;
  private AHRS navx = new AHRS(SPI.Port.kMXP);

 
  public Drivetrain() {
    leftDriveTalon = new WPI_TalonSRX(Constants.DriveTrainConstants.leftDriveTalonConstants);
    rightDriveTalon = new WPI_TalonSRX(Constants.DriveTrainConstants.rightDriveTalonConstants);
    leftDriveTalon.setNeutralMode(NeutralMode.Coast);
    rightDriveTalon.setNeutralMode(NeutralMode.Coast);
    rightDriveTalon.setInverted(true);
    leftDriveTalon.setSensorPhase(true);
    rightDriveTalon.setSensorPhase(true);

    leftDriveTalon.configFactoryDefault();
    leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightDriveTalon.configFactoryDefault();
    rightDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
  }

 public void TankDrive(double leftSpeed, double rightSpeed) {
  leftDriveTalon.set(leftSpeed);
  rightDriveTalon.set(rightSpeed);
 }

 public double GetCurrentAngle() {
  double currentAngle = -navx.getAngle();
  return currentAngle;
 }

 public void resetGyro() {
  navx.reset();
 }

 public double getTicks() {
  double leftPosition = leftDriveTalon.getSelectedSensorPosition(0);
  double rightPosition = rightDriveTalon.getSelectedSensorPosition(0);
  double avgPosition = (leftPosition + rightPosition)/2;
  return avgPosition;
}

public double getMeters() {
  double metersPerTick = (0.1524 * 3.1415926535)/4096;
  double positionInMeters = getTicks() * metersPerTick;
  return positionInMeters;
}

public void resetEncoders() {
  leftDriveTalon.getSelectedSensorPosition();
  rightDriveTalon.getSelectedSensorPosition();
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Voltage", leftDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Voltage", rightDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Angle", navx.getAngle());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

