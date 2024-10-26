// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.text.html.parser.DTD;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class TeleOp extends Command {

  Joystick js = new Joystick(2);
  Drivetrain dr;
  public TeleOp(Drivetrain drivetrain) {
    dr = drivetrain;
    addRequirements(dr);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dr.TankDrive(0,0);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double leftPowerRaw = js.getRawAxis(1);
    final double rightPowerRaw = js.getRawAxis(5);
    dr.TankDrive(leftPowerRaw, rightPowerRaw);
    SmartDashboard.putNumber("leftSpeed", leftPowerRaw);
    SmartDashboard.putNumber("rightSpeed", rightPowerRaw);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
