// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathPlannerLoader extends SequentialCommandGroup {
  /** Creates a new TestPathPlanner01. */

  
  public PathPlannerLoader(DriveSubsystem m_robot, String fileName) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PathPlannerTrajectory examplePath = PathPlanner.loadPath(fileName, new PathConstraints(1, 0.5));


    addCommands(
      m_robot.followTrajectoryCommand(examplePath, true)
    );
  }
}
