package frc.robot.commands.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
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

      SmartDashboard.putBoolean("ResetShoulderEncoder", false);
    }

  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {


      if(Math.abs(MathUtil.applyDeadband(-Robot.getArmControlJoystick().getRightY(), .01))>0){//the joystick is used to make microadjustments
        shoulder.setPower(MathUtil.applyDeadband(-Robot.getArmControlJoystick().getRightY(), .06)/5.0);
        shoulderPos = shoulder.getPosition();
      }
      else{//otherwise hold the current position
        shoulder.goToPos(shoulderPos);
      }

      if(SmartDashboard.getBoolean("ResetShoulderEncoder", false)){
        shoulder.resetEncoder();
        shoulderPos = 0;
        SmartDashboard.putBoolean("ResetShoulderEncoder", false);
      }
      


        //Dpad up sets shoulder to up position
        if(Robot.getArmControlJoystick().getPOV()==0){
          shoulderPos = SmartDashboard.getBoolean("Cone/Cube", isFinished())?Constants.ShoulderConstants.shoulderMidCube:Constants.ShoulderConstants.shoulderMidCone;
        }
        if(Robot.getArmControlJoystick().getPOV()==90){
          shoulderPos = SmartDashboard.getBoolean("Cone/Cube", isFinished())?Constants.ShoulderConstants.shoulderFarCube:Constants.SliderConstants.sliderFarCone;

        }
        else if(Robot.getArmControlJoystick().getYButton()){
          shoulderPos = Constants.ShoulderConstants.shoulderGround;
        }
        SmartDashboard.putNumber("DPAD:", Robot.getArmControlJoystick().getPOV());

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
