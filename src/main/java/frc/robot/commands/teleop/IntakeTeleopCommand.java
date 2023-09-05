package frc.robot.commands.teleop;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeTeleopCommand extends CommandBase{
    private IntakeSubsystem intake;


    public IntakeTeleopCommand(IntakeSubsystem intake){
        this.intake =intake;
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(Robot.getArmControlJoystick().getBButton()){
      intake.setPower(0.75);                      // Mr. Michaud update. Don't run at full power
    }
    else if(Robot.getArmControlJoystick().getXButton()){
      intake.setPower(-0.75);                           // Mr. Michaud update. Don't run at full power

    }
    else{
      intake.setPower(0);
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
