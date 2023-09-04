package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.SliderSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class SubsystemEncoderTuning extends CommandBase{


    private ShoulderSubsystem shoulder;
    private SliderSubsystem slider;
    private WristSubsystem wrist;

    private double shoulderPos, sliderPos, wristPos = 0;

    /** Creates a new DriveCommand. */
  public SubsystemEncoderTuning(ShoulderSubsystem shoulder, SliderSubsystem slider, WristSubsystem wrist){
    this.shoulder = shoulder;
    this.slider = slider;
    this.wrist = wrist;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shoulder.resetEncoder();
    slider.resetEncoder();
    wrist.resetEncoder();
    SmartDashboard.putNumber("SliderTargetPos", 0);
    SmartDashboard.putNumber("ShoulderTargetPos", 0);
    SmartDashboard.putNumber("WristTargetPos", 0);
    SmartDashboard.putBoolean("SendValues", false);//Set button to be  in false state
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(SmartDashboard.getBoolean("SendValues", false)){//If the sendValues button has been pressed/is in true state
      //Retreive Desired States
        sliderPos = SmartDashboard.getNumber("SliderTargetPos", sliderPos);
        shoulderPos = SmartDashboard.getNumber("ShoulderTargetPos", shoulderPos);
        wristPos = SmartDashboard.getNumber("WristTargetPos", wristPos);
        //Set Shoulder, Slider and Wrist to desired States
        shoulder.goToPos(shoulderPos);
        slider.goToPos(sliderPos);
        wrist.goToPosition(wristPos);
        //Reset the button to false state
        SmartDashboard.putBoolean("SendValues", false);
    }

  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
    
}
