package frc.robot.commands.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderTeleopCommand extends CommandBase{
    
    private ShoulderSubsystem shoulder;
    private double shoulderPos;
    private int shoulderMode; //0 is power 1 is encoder
    public ShoulderTeleopCommand(ShoulderSubsystem shoulder){
        this.shoulder = shoulder;
        this.shoulderPos=0;
        this.shoulderMode = 0;

    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      shoulderPos = shoulder.getPosition();
    }

  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        if(shoulderMode == 0){
          shoulder.setPower(MathUtil.applyDeadband(-Robot.getArmControlJoystick().getRightY(), .06)/5.0);
          //update the shoulderPose with the current state of the shoulder so it accurately tracks where the shoulder is when controlled by power
          shoulderPos = shoulder.getPosition();
          SmartDashboard.putNumber("ShoulderPosExpected",shoulderPos);
          SmartDashboard.putString("ShoulderMode", "Power");
          

        }
        else{
          shoulderPos += MathUtil.applyDeadband(-Robot.getArmControlJoystick().getRightY(), .1)/5.0;
         // shoulderPos = MathUtil.clamp(shoulderPos, Constants.ShoulderConstants.MIN_ENCODER_POS, Constants.ShoulderConstants.MAX_ENCODER_POS);
          SmartDashboard.putNumber("ShoulderPosExpected", shoulderPos);
          SmartDashboard.putString("ShoulderMode", "Encoder");
          shoulder.goToPos(shoulderPos);
        }

        //Changing shoulderMode
        if(!Robot.getArmControlJoystick().getRightStickButton() && Robot.getArmControlJoystick().getRightBumperPressed()){
          if(shoulderMode==0)shoulderMode=1;
          else shoulderMode=0;
        }

        //System.out.println(Robot.getArmControlJoystick().getRightBumperPressed())
        //Resetting the encoder for the shoulder; happens if right bumper is pressed while joystick button is held
        if(Robot.getArmControlJoystick().getRightBumperPressed() && Robot.getArmControlJoystick().getRightStickButton()){
          shoulder.resetEncoder();
          shoulderPos = 0;
          SmartDashboard.putNumber("ShoulderPosExpected", shoulderPos);
        }
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
