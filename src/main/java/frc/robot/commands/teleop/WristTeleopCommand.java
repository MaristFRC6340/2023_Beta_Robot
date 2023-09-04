package frc.robot.commands.teleop;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.WristSubsystem;

public class WristTeleopCommand extends CommandBase{
    private WristSubsystem wrist;//wrist subsystem object
    private double wristPos = 0;//a double variable which holds the desired encoder position of the wrist


    public WristTeleopCommand(WristSubsystem wrist){
        this.wrist =wrist;
    }

    // Called when the command is initially scheduled.
    //Wil be called whenever the robot is re-enabled in teleop mode not just the first time
  @Override
  public void initialize() {
    wristPos = wrist.getPosition();//When we start teleop we want our desired wrist position to be whatever our wrist encoder reads at. 
    //This prevents the wrist from jumping up if its encoder position is initially different from our desired posiion
  }
  

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Increase or decrease the desired Wrist Position by a very small amount based on triggers; (The wrist's encoder range without running into the slider is from 0-25ish)
    wristPos += Robot.getArmControlJoystick().getRightTriggerAxis()*(1/5.0);//right trigger raises
    wristPos -= Robot.getArmControlJoystick().getLeftTriggerAxis()*(1/5.0);//left triggers lowers
    SmartDashboard.putNumber("Wris Pos Expected", wristPos);//Log Desired Encoder Value o Smart Dashboard
    wrist.goToPosition(wristPos);//set the wrist to go to the desired position
    //Old code used to move wrist based on power from triggers; This can be readded back later for a manual control if we ever need to reset encoders
    //wrist.setPower((Robot.getArmControlJoystick().getRightTriggerAxis()-Robot.getArmControlJoystick().getLeftTriggerAxis())*1.5);

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;//since this is a teleop command it never ends
  }
  
}
