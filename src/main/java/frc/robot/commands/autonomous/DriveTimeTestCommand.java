// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This is a test of using Swerve Drive .drive function with time
// Short direct driving given inputs of deltaX and deltaY simulating Remote
// Base for driving based on sensors
// Backup for Elemental Autos
// Future Task: Get any type of input reading from encoders to estimate distance traveled
// michaudc 20 Feb 23

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTimeTestCommand extends CommandBase {

  // Field for Drivetrain
  private DriveSubsystem m_robot;

  // Field for Wrist
  

  // Fields for motion control
  private double deltaX;
  private double deltaY;
  private double theta;
  private double time;

  // Fields for Time Control
  private long startTime = 0;
  private long endTime = 0;
  private double duration = 0;


  /** Creates a new DriveTimeTestCommand. */
  public DriveTimeTestCommand(DriveSubsystem swerve, double deltaX, double deltaY, double theta, double duration) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_robot = swerve;

    this.deltaX = deltaX;
    this.deltaY = deltaY;
    this.theta = theta;
    this.duration = duration;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    endTime = startTime + (long)(duration*1000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Compute Vectors based on any sensor input
    // For now - direct transfer of fields from constructor input
    double dX = deltaX;
    double dY = deltaY;
    double dTheta = theta;

    // Drive Robot: Field Oriented to False
    m_robot.drive(dX, dY, dTheta, false);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the Robot
    m_robot.drive(0, 0, 0, false);
    m_robot.setX();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    long currentTime = System.currentTimeMillis();
    if (currentTime < endTime) {
      return false;
    }
    else {
      return true;
    }
  }
}