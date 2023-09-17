// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autonomous.RampClimbNavX;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.SliderSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/åstable/docs/software/commandbased/convenience-features.html
public class AutoRampClimbPlusHighCubeCommand extends SequentialCommandGroup {
  /** Creates a new AutoRampClimb. */
  public AutoRampClimbPlusHighCubeCommand(DriveSubsystem drive, ShoulderSubsystem shoulder, WristSubsystem wrist, SliderSubsystem slider, IntakeSubsystem intake) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //intake power is -.5 to outtake a cube
      new HighCubeAndResetCommand(wrist, shoulder, slider, intake, -.5),
      new AutoRampClimb(drive)
      
    );
  }
}