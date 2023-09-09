package frc.robot.commands.teleop;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.WristSubsystem;

public class WristTeleopCommand extends CommandBase{
    private WristSubsystem wrist;
    private double wristPos = 0;//holds the desired encoder position of the writ
    private int wristMode = 0;//If wrist mode is 0 we move by manual power; If wrist mode is 1 we move using encoder counts


    public WristTeleopCommand(WristSubsystem wrist){
        this.wrist =wrist;
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristPos = wrist.getPosition();
    SmartDashboard.putBoolean("ResetWristEncoder", false);

  }
  

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    


    wristPos += Robot.getArmControlJoystick().getRightTriggerAxis()*(1/5.0);
    wristPos -= Robot.getArmControlJoystick().getLeftTriggerAxis()*(1/5.0);
    
    SmartDashboard.putNumber("Wris Pos Expected", wristPos);
    wrist.goToPosition(wristPos);
    //wrist.setPower((Robot.getArmControlJoystick().getRightTriggerAxis()-Robot.getArmControlJoystick().getLeftTriggerAxis())*1.5);


    if(SmartDashboard.getBoolean("ResetWristEncoder", false)){
      wrist.resetEncoder();
      wristPos = 0;
      SmartDashboard.putBoolean("ResetWristEncoder", false);
    }

    if(Robot.getArmControlJoystick().getPOV()==0){
      wristPos = Constants.WristConstants.outtakeMidPoleCone;
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
