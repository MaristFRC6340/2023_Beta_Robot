// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.commands.autonomous.RampClimbNavX;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.SliderSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeCommand extends CommandBase{


    IntakeSubsystem intake;
    double power;
    double duration;
    double startTime;
    double endTime;
  
  /** Creates a new AutoRampClimb. */
  public IntakeCommand(IntakeSubsystem wrist, double power, double duration) {
    this.intake = intake;
    this.power =power;
    this.duration = duration;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
      startTime = System.currentTimeMillis();
      endTime = startTime + (long)(duration*1000);

  }

  @Override
  public void execute() {
      intake.setPower(power);
      
  }

  @Override
  public void end(boolean interrupted) {
      intake.setPower(0);
  }

  @Override
  public boolean isFinished() {
      return System.currentTimeMillis() < endTime;
  }


}