// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.commands.autonomous.RampClimbNavX;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.SliderSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetSubsystemsCommand extends CommandBase{


    ShoulderSubsystem shoulder;
    SliderSubsystem slider;
    WristSubsystem wrist;
    boolean isFinished = false;
  /** Creates a new AutoRampClimb. */
  public ResetSubsystemsCommand(WristSubsystem wrist, ShoulderSubsystem shoulder, SliderSubsystem slider) {
    this.slider = slider;
    this.wrist =wrist;
    this.shoulder = shoulder;
    

    addRequirements(wrist, shoulder, slider);
  }

  @Override
  public void initialize() {
      
  }

  @Override
  public void execute() {
      if(!isFinished&& Math.abs(wrist.getPosition()-Constants.WristConstants.wristRest)>.1 || Math.abs(shoulder.getPosition()-Constants.ShoulderConstants.shoulderGround)>.1 || Math.abs(slider.getPosition()-Constants.SliderConstants.sliderGround)>.1){
        wrist.goToPosition(Constants.WristConstants.wristRest);
        slider.goToPos(Constants.SliderConstants.sliderGround);
        shoulder.goToPos(Constants.ShoulderConstants.shoulderGround);
      }
      else{
        isFinished = true;
      }
      
  }

  @Override
  public void end(boolean interrupted) {
      
  }

  @Override
  public boolean isFinished() {
      return isFinished;
  }


}