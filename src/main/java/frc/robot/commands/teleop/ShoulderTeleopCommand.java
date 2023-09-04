package frc.robot.commands.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderTeleopCommand extends CommandBase{
    
    private ShoulderSubsystem shoulder;
    private double shoulderPos;//Desired Position of the Shoulder
    private int shoulderMode; //Mode that the shoulder is being controlled in; 0 is direct power 1 is through encoder positions
    public ShoulderTeleopCommand(ShoulderSubsystem shoulder){
        this.shoulder = shoulder;
        this.shoulderPos=0;
        this.shoulderMode = 0;

    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      shoulderPos = shoulder.getPosition();//When we start teleop we want our desired shoulder position to be whatever our shoulder encoder reads at. 
      //This prevents the shoulder from jumping up if its encoder position is initially different from our desired posiion
    }

  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        /**
       * There are two ways to control the shoulder: direct power, and encoder positions.
       * 
       * Encoder Positions allow us to have the motors hold the shoulder at a certain position despite any forces of weight 
       * or gravity put on it. However, for fine control with the joysticks it isnt the best because if you are constantly telling the shoulder to run to a postition just a bit farther away
       * the shoulder motors are going to groan a little. Encoder is best used for running to set Posiions or if  you want to hold your position
       * 
       * Direct Power  relies on the motors being on brake mode to hold the position, but it is good for fine motor adjustment. It basically
       * just sends positive power to the motor if the joystick is pushed up and negatiev power if it is pushed down
       * 
       * In the coming weeks we will problay add linked encoder positions to buttons and use the encoder mode on the joysticks for slight manual adjustment
       * Direct power can be used if we need to bring the shoulder to a certain position to reset the encoders
       * 
       * 
       */
        if(shoulderMode == 0){//If we are sending power directly to the motors

          //Send the power based off on the y axis of the robot's right joystick
          shoulder.setPower(MathUtil.applyDeadband(-Robot.getArmControlJoystick().getRightY(), .06)/5.0);
          //update the shoulderPose with the current state of the shoulder so it accurately tracks where the shoulder is when controlled by power
          shoulderPos = shoulder.getPosition();
          SmartDashboard.putString("ShoulderMode", "Power");
          

        }
        else{//If we are making the slider run to an encoder position 

          //Increase or Decrease the desired encoder position based off of the yaxis robot's left joystick
          shoulderPos += MathUtil.applyDeadband(-Robot.getArmControlJoystick().getRightY(), .1)/5.0;
          /**Potential clamping shoulder positions for min anx max positions (Not sure if we want to do this or not) */
         // shoulderPos = MathUtil.clamp(shoulderPos, Constants.ShoulderConstants.MIN_ENCODER_POS, Constants.ShoulderConstants.MAX_ENCODER_POS);
          SmartDashboard.putString("ShoulderMode", "Encoder");
          shoulder.goToPos(shoulderPos);
        }

        //Change the Shoulder Mode by pressing the right bumper on the Gamepad. The right Stick Button can not be pressed at the same time because that is the controls for resetting the encoder
        if(!Robot.getArmControlJoystick().getRightStickButton() && Robot.getArmControlJoystick().getRightBumperPressed()){
          if(shoulderMode==0)shoulderMode=1;
          else shoulderMode=0;
        }

        //Resetting the encoder for the slider by  pressing right bumper while right joystick button is held
        if(Robot.getArmControlJoystick().getRightBumperPressed() && Robot.getArmControlJoystick().getRightStickButton()){
          shoulder.resetEncoder();//reset encoder and desired shoulder position
          shoulderPos = 0;
        }

        SmartDashboard.putNumber("ShoulderPosExpected", shoulderPos);

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
