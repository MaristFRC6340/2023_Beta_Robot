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
        
        // intake power is .5 to intake a cone
        new MidConeAndResetCommand(wrist, shoulder, slider, intake, .5),
        new DriveTimeTestCommand(drive, -.2, 0, 0, 4.5)
        
      );
    }
  }