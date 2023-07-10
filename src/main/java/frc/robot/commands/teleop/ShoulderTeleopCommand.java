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
    public ShoulderTeleopCommand(ShoulderSubsystem shoulder){
        this.shoulder = shoulder;
        this.shoulderPos=0;
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      shoulder.resetEncoder();
    }

  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shoulderPos += MathUtil.applyDeadband(-Robot.getArmControlJoystick().getRightY(), .1)/10.0;
        shoulderPos = MathUtil.clamp(shoulderPos, Constants.ShoulderConstants.MIN_ENCODER_POS, Constants.ShoulderConstants.MAX_ENCODER_POS);
        SmartDashboard.putNumber("ShoulderPosExpected", shoulderPos);
        shoulder.goToPos(shoulderPos);
        // shoulder.setPower(MathUtil.applyDeadband(-Robot.getArmControlJoystick().getRightY(), .06));
        // SmartDashboard.putNumber("ShoulderPosExpected", -Robot.getArmControlJoystick().getRightY());

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
