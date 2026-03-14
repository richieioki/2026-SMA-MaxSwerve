// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PointToPointPID extends Command {
  private PIDController X = new PIDController(2, 0, 0);
  private PIDController Y = new PIDController(2, 0, 0);
  private PIDController A = new PIDController(0.05, 0, 0);
  DriveSubsystem driveSubsystem;
  Pose2d setpoint;
  /** Creates a new PointToPointPID. */
  public PointToPointPID(DriveSubsystem drive, Pose2d setpoint) {
    driveSubsystem = drive;
    addRequirements(drive);
    this.setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    X.setSetpoint(setpoint.getX());
    X.setTolerance(0.001);
    Y.setSetpoint(setpoint.getY());
    Y.setTolerance(0.001);
    A.setSetpoint(setpoint.getRotation().getDegrees());
    A.setTolerance(0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVal = X.calculate(driveSubsystem.getPose().getX());
    double yVal = Y.calculate(driveSubsystem.getPose().getY());
    double aVal = A.calculate(driveSubsystem.getPose().getRotation().getDegrees());

    driveSubsystem.drive(xVal, yVal, aVal, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return X.atSetpoint() && Y.atSetpoint() && A.atSetpoint();
  }
}
