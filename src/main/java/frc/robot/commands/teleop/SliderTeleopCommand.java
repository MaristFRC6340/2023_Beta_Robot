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
    }

  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      /**
       * There are two ways to control the slider: direct power, and encoder positions.
       * 
       * Encoder Positions allow us to have the motors hold the slider at a certain position despite any forces of weight 
       * or gravity put on it. However, for fine control with the joysticks it isnt the best because if you are constantly telling the slider to run to a postition just a bit farther away
       * the slider motors are going to groan a little. Encoder is best used for running to set Posiions or if  you want to hold your position
       * 
       * Direct Power  relies on the motors being on brake mode to hold the position, but it is good for fine motor adjustment. It basically
       * just sends positive power to the motor if the joystick is pushed up and negatiev power if it is pushed down
       * 
       * In the coming weeks we will problay add linked encoder positions to buttons and use the encoder mode on the joysticks for slight manual adjustment
       * Direct power can be used if we need to bring the slider to a certain position to reset the encoders
       * 
       * 
       */
        

        if(sliderMode == 0){//If we are sending power directly to the motors
          //Send th power based off on the y axis of the robot's left joystick
          slider.setPower(MathUtil.applyDeadband(-Robot.getArmControlJoystick().getLeftY(), .06)/5.0);
          SmartDashboard.putString("SliderMode", "Power");//Update the Slider Mode on Smart Dashboard

          //update the sliderPos with the current state of the slider so it accurately tracks where the slider is when controlled by power
          sliderPos = slider.getPosition();
        }
        else{//If we are making the slider run to an encoder position 

          //Increase or Decrease the desired encoder position based off of the yaxis robot's left joystick
          sliderPos += MathUtil.applyDeadband(-Robot.getArmControlJoystick().getLeftY(), .1)/5.0;
          SmartDashboard.putString("SliderMode", "Encoder");//update the Slder Mode on Smart Dashboard


          /**Potential Slider Position Clamping for Min and Max Positions. (Not sure if we want to use this yet) */
          //sliderPos = MathUtil.clamp(sliderPos, Constants.SliderConstants.MIN_ENCODER_POS, Constants.SliderConstants.MAX_ENCODER_POS);
          
          slider.goToPos(sliderPos);//set the slider to run to the desired slider Position
        }


        //Change the SLider Mode by pressing the left bumper on the Gamepad. The Left Stick Button can not be pressed at the same time because that is the controls for resetting the encoder
        if(!Robot.getArmControlJoystick().getLeftStickButton() && Robot.getArmControlJoystick().getLeftBumperPressed()){
          if(sliderMode==0)sliderMode=1;
          else sliderMode=0;
        }

        //Resetting the encoder for the slider by  pressing left bumper while left joystick button is held
        if(Robot.getArmControlJoystick().getLeftBumperPressed() && Robot.getArmControlJoystick().getLeftStickButton()){
          slider.resetEncoder();//reset slide Encoder
          sliderPos = 0;//reset slider desired position
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
