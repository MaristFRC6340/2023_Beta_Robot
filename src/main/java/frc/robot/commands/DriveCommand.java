// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {

  // Field for DriveSubsystem
  private final DriveSubsystem m_robotDrive;


  /** Creates a new DriveCommand. */
  public DriveCommand(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_robotDrive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Updated Drive Command
    m_robotDrive.drive(
                MathUtil.applyDeadband(-Robot.getDriveControlJoystick().getLeftY()*DriveConstants.SpeedMultiplier, 0.06),
                MathUtil.applyDeadband(-Robot.getDriveControlJoystick().getLeftX()*DriveConstants.SpeedMultiplier, 0.06),
                MathUtil.applyDeadband(-Robot.getDriveControlJoystick().getRightX()*DriveConstants.SpeedMultiplier, 0.06),
                true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Todo: Set motors to stop
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
