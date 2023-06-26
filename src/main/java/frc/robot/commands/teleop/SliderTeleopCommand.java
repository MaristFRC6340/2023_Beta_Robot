package frc.robot.commands.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.SliderSubsystem;

public class SliderTeleopCommand extends CommandBase{
    
    private SliderSubsystem slider;
    private double sliderPos;
    public SliderTeleopCommand(SliderSubsystem slider){
        this.slider = slider;
        this.sliderPos=0;
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        sliderPos -= Robot.getArmControlJoystick().getLeftY();
        sliderPos = MathUtil.clamp(sliderPos, Constants.SliderConstants.MIN_ENCODER_POS, Constants.SliderConstants.MAX_ENCODER_POS);
        SmartDashboard.putNumber("sliderPosExpected", sliderPos);
        slider.goToPos(sliderPos);

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
