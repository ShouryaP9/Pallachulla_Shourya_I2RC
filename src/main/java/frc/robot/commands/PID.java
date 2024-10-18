// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class PID extends Command {
  public Drivetrain drive;
  public double setPointAngle = 90;
  public PIDController controller = new PIDController(1, 0, 0);
  /** Creates a new PID. */
  public PID(Drivetrain dr, double wantedAngle, PIDController pidcontroller) {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = dr;
    setPointAngle = wantedAngle;
    controller = pidcontroller;
    addRequirements(dr);
    controller.setTolerance(5);

  }

  public double chassisAngle() {
    double currentAngle = drive.GetCurrentAngle();
    return currentAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetGyro();
    drive.TankDrive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = controller.calculate(chassisAngle(), setPointAngle);
    drive.TankDrive(output * -1, output * -1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.TankDrive(0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
