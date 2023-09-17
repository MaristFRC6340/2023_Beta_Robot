// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import frc.robot.subsystems.SliderSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SliderRetractCommand extends CommandBase{


    
    SliderSubsystem slider;
    boolean isFinished = false;
  /** Creates a new AutoRampClimb. */
  public SliderRetractCommand( SliderSubsystem slider) {
    this.slider = slider;
    addRequirements( slider);
  }

  @Override
  public void initialize() {
      
  }

  @Override
  public void execute() {
      if(!isFinished&& Math.abs(slider.getPosition()-Constants.SliderConstants.sliderGround)>.1){
        slider.goToPos(Constants.SliderConstants.sliderGround);
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