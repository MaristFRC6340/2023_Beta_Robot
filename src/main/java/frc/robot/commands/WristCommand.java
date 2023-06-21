package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.WristSubsystem;

public class WristCommand extends CommandBase{
    private WristSubsystem wrist;
    private double wristPos = 0;


    public WristCommand(WristSubsystem wrist){
        this.wrist =wrist;
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wristPos += Robot.getArmControlJoystick().getRightTriggerAxis();
    wristPos -= Robot.getArmControlJoystick().getLeftTriggerAxis();
    wristPos = MathUtil.clamp(wristPos, 0, 400);
    SmartDashboard.putNumber("WRISTPOSJOYSTICK", wristPos);
    wrist.goToPosition(wristPos);

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
