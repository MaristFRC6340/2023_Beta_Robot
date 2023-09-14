package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SliderConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.SliderSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class AutoBlueMidCone extends SequentialCommandGroup {
    /** Creates a new AutoRampClimb. */
    public AutoBlueMidCone(WristSubsystem wrist, ShoulderSubsystem shoulder, SliderSubsystem slider, IntakeSubsystem intake, DriveSubsystem drive) {
  
      // Add your commands in the addCommands() call, e.g.
      // addCommands(new FooCommand(), new BarCommand());
      addCommands(
        
        // Ramp Climber: Substitute with RampClimbNavX when finished
        
        new MidConeCommand(wrist, shoulder, slider),
        new WaitCommand(.5),
        new IntakeCommand(intake, .5,.5),
        new WaitCommand(1),
        new ResetSubsystemsCommand(wrist, shoulder, slider),
        new WaitCommand(1),
        new DriveTimeTestCommand(drive, -.2, 0, 0, 4.5)
        
      );
    }
  }