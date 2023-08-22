// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTeleopCommand extends CommandBase {

  // Field for DriveSubsystem
  private final DriveSubsystem m_robotDrive;
  private  double manualSpeedModifier;

  private double desiredHeading;
  private PIDController thetaController;


  /** Creates a new DriveCommand. */
  public DriveTeleopCommand(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_robotDrive = drive;
    manualSpeedModifier = 1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //set the deisred heading to the odometrys heading
    desiredHeading = m_robotDrive.getPose().getRotation().getDegrees();
    //intialize the PID Controller
    thetaController = new PIDController(Constants.DriveConstants.thetaP, Constants.DriveConstants.thetaI, Constants.DriveConstants.thetaD);
    thetaController.enableContinuousInput(0, 360);
  }

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

    //update the theta Controller
    thetaController.setSetpoint(desiredHeading);

    if(Math.abs(MathUtil.applyDeadband(-Robot.getDriveControlJoystick().getRightX(), 0.06)) > 0){
      desiredHeading += MathUtil.applyDeadband(Robot.getDriveControlJoystick().getRightX(), 0.06);
    }

    //wrap the values for the desired heading
    if(desiredHeading > 360)desiredHeading = desiredHeading-360;
    else if(desiredHeading <=0) desiredHeading = 360 + desiredHeading;



    //get the power to run the rotational speed at
    double rotPower = thetaController.calculate(m_robotDrive.getPose().getRotation().getDegrees());

    //Send values to SmartDashboard
    SmartDashboard.putNumber("rotPower", rotPower);
    SmartDashboard.putNumber("desiredHeading", desiredHeading);




    // Updated Drive Command
    // Postive and Negative signs fixed for Remote input: michaudc 05 may 23
    m_robotDrive.drive(
                MathUtil.applyDeadband(-Robot.getDriveControlJoystick().getLeftY()*DriveConstants.SpeedMultiplier*manualSpeedModifier, 0.06),
                MathUtil.applyDeadband(-Robot.getDriveControlJoystick().getLeftX()*DriveConstants.SpeedMultiplier*manualSpeedModifier, 0.06),
                MathUtil.applyDeadband(-Robot.getDriveControlJoystick().getRightX()*DriveConstants.SpeedMultiplier*manualSpeedModifier, 0.06),
                /**rotPower,**/
              true);

    if(Robot.getDriveControlJoystick().getPOV()!=-1){
      m_robotDrive.zeroHeading(Robot.getDriveControlJoystick().getPOV());
      //set the desired heading to be the POV
      desiredHeading = Robot.getDriveControlJoystick().getPOV();
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
