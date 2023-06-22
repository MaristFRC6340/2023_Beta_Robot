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
  private  double manualSpeedModifier;


  /** Creates a new DriveCommand. */
  public DriveCommand(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_robotDrive = drive;
    manualSpeedModifier = 1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Robot.getDriveControlJoystick().getLeftStickButton()){
      manualSpeedModifier = 2;
    }
    else if (Robot.getDriveControlJoystick().getLeftTriggerAxis()>0){
      manualSpeedModifier =1-Robot.getDriveControlJoystick().getLeftTriggerAxis(); 
    }
    else{
      manualSpeedModifier=1;
    }
    // Updated Drive Command
    // Postive and Negative signs fixed for Remote input: michaudc 05 may 23
    m_robotDrive.drive(
                MathUtil.applyDeadband(-Robot.getDriveControlJoystick().getLeftY()*DriveConstants.SpeedMultiplier*manualSpeedModifier, 0.06),
                MathUtil.applyDeadband(-Robot.getDriveControlJoystick().getLeftX()*DriveConstants.SpeedMultiplier*manualSpeedModifier, 0.06),
                MathUtil.applyDeadband(-Robot.getDriveControlJoystick().getRightX()*DriveConstants.SpeedMultiplier*manualSpeedModifier, 0.06),
              true);

    if(Robot.getDriveControlJoystick().getPOV()!=-1){
      m_robotDrive.zeroHeading(Robot.getDriveControlJoystick().getPOV());
    }

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
