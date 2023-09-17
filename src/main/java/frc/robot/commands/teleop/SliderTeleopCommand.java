package frc.robot.commands.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.SliderSubsystem;

public class SliderTeleopCommand extends CommandBase{
    
    private SliderSubsystem slider;
    private double sliderPos;//Desired Position of the Slider
    private int sliderMode;//Mode that the slider is being controlled in; 0 is direct power 1 is through encoder positions
    public SliderTeleopCommand(SliderSubsystem slider){
        this.slider = slider;
        this.sliderPos=0;
        this.sliderMode=0;
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      sliderPos = slider.getPosition();//When we start teleop we want our desired slider position to be whatever our slider encoder reads at. 
      //This prevents the slider from jumping up if its encoder position is initially different from our desired posiion
      SmartDashboard.putBoolean("ResetSliderEncoder", false);
    }

  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
     
      if(Math.abs(MathUtil.applyDeadband(-Robot.getArmControlJoystick().getLeftY(),.02))>0){//the joystick is used to make microadjustments
        slider.setPower(MathUtil.applyDeadband(-Robot.getArmControlJoystick().getLeftY(), .06)/5.0);
        sliderPos = slider.getPosition();
      }
      else{//otherwise hold the current position
        slider.goToPos(sliderPos);
      }

      if(SmartDashboard.getBoolean("ResetSliderEncoder", false)){
        slider.resetEncoder();
        sliderPos = 0;
        SmartDashboard.putBoolean("ResetSliderEncoder", false);
      }

      if(Robot.getArmControlJoystick().getPOV()==0){
        //False if Slider true if cube
        sliderPos = Constants.SliderConstants.sliderMidCone;
      }
      if(Robot.getArmControlJoystick().getPOV()==90){
        sliderPos = Constants.SliderConstants.sliderFarCube;
      }
      else if(Robot.getArmControlJoystick().getPOV()==180){
        sliderPos = Constants.SliderConstants.sliderGround;
      }
      if(Robot.getArmControlJoystick().getPOV ()==270){
        sliderPos = Constants.SliderConstants.sliderConePickup;
      }
      else if(Robot.getArmControlJoystick().getYButton()){
        sliderPos = Constants.SliderConstants.sliderGround;
      }

        //Send the Desired Slider Position to Smart DashBoard
        SmartDashboard.putNumber("sliderPosExpected", sliderPos);

    }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;//command never ends because it is a teleop command
  }
}
