// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Robot;

public class ArmCommand extends CommandBase {
  /** Creates a new ArmCommand. */

  private final ArmSubsystem arm;
  double leftY = 0, rightY = 0;
  double leftX = 0, rightX = 0;

  public ArmCommand( ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Robot.getArmControlJoystick().getRawButton(5)){
      arm.armUp();
    }
    if(Robot.getArmControlJoystick().getRawButton(3)){
      arm.armDown();
    }

    //recent
    if(Robot.getArmControlJoystick().getRawButton(4)){
      arm.openLeft();
      arm.closeRight();
    }
    if(Robot.getArmControlJoystick().getRawButton(6)){
      arm.closeLeft();
      arm.openRight();
    }
    
    if(Robot.getArmControlJoystick().getRawButton(4)){
      arm.setWristMotorPower(.5);
      arm.setIntakeLeftMotorPower(.5);

    }
    if(Robot.getArmControlJoystick().getRawButton(1)){
      arm.setWristMotorPower(-.5);
      arm.setIntakeLeftMotorPower(-.5);
    }
    if(Robot.getArmControlJoystick().getRawButton(2)){
      arm.setIntakeLeftMotorPower(0); //stop button b
    }

    leftY = Robot.getArmControlJoystick().getRawAxis(1);
    rightY = Robot.getArmControlJoystick().getRawAxis(5);

    rightX = Robot.getArmControlJoystick().getRawAxis(4);
    leftX = Robot.getArmControlJoystick().getRawAxis(0);
    
    arm.setArmLengthMotorPower(MathUtil.applyDeadband(leftY, 0.06));
    arm.setWristMotorPower(MathUtil.applyDeadband(rightY, 0.06));
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
