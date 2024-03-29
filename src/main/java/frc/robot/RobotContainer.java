// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.SliderSubsystem;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import frc.robot.commands.autonomous.AutoBlueMidCone;
import frc.robot.commands.autonomous.AutoRampClimb;
import frc.robot.commands.autonomous.AutoRampClimbExitCommand;
import frc.robot.commands.autonomous.AutoRampClimbPlusHighCubeCommand;
import frc.robot.commands.autonomous.AutoRampClimbPlusMidCubeCommand;
import frc.robot.commands.autonomous.IntakeCommand;
import frc.robot.commands.autonomous.KyleSabatogeJavaPath;
import frc.robot.commands.autonomous.MidConeCommand;
import frc.robot.commands.autonomous.PIDTuningTestPaths;
import frc.robot.commands.autonomous.PathPlannerLoader;
import frc.robot.commands.autonomous.ResetSubsystemsCommand;
import frc.robot.commands.autonomous.StraightLineAuto;
import frc.robot.commands.autonomous.TestPathPlanner;
import frc.robot.commands.teleop.DrivePIDTuning;
import frc.robot.commands.teleop.DriveTeleopCommand;
import frc.robot.commands.teleop.SliderTeleopCommand;
import frc.robot.commands.teleop.SubsystemEncoderTuning;
import frc.robot.commands.teleop.WristTeleopCommand;
import frc.robot.commands.teleop.ShoulderTeleopCommand;
import frc.robot.commands.teleop.IntakeTeleopCommand;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem robotDrive = new DriveSubsystem();
  private final WristSubsystem wrist = new WristSubsystem();
  private final SliderSubsystem slider = new SliderSubsystem();
  private final ShoulderSubsystem shoulder = new ShoulderSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();



  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    //reset the subsystem encoders
    resetSubsystemEncoders();

    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(Robot.getDriveControlJoystick(), Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> robotDrive.setX(),
            robotDrive));  
  }

  /**
   * Call this method once upon the RobotContainer's creation to reset the encoders of the wrist, slider, and shoulder back to 0
   * @return
   */
  public void resetSubsystemEncoders(){
    shoulder.setEncoder(Constants.ShoulderConstants.shoulderStartPosition);
    slider.resetEncoder();
    wrist.resetEncoder();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        robotDrive::setModuleStates,
        robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> robotDrive.drive(0, 0, 0, false));
  }
  //TODO: WE can probaly delete all these and just create the Command objects in the Robot class
  public Command getWristTeleopCommand(){
    return new WristTeleopCommand(wrist);  // Placeholder
  }
  public Command getDriveTeleopCommand() {
    return new DriveTeleopCommand(robotDrive);
  }
  public Command getSliderTeleopCommand(){
    return new SliderTeleopCommand(slider);
  }
  public Command getShoulderTeleopCommand(){
    return new ShoulderTeleopCommand(shoulder);
  }
  public Command getIntakeTeleopCommand(){
    return new IntakeTeleopCommand(intake);
  }

  public Command getTestPathCommand() {
    return new TestPathPlanner(robotDrive);
  }
  public Command getStraightLineAuto(){
    return new StraightLineAuto(robotDrive);
  }
  public Command getKyleSabatogeCommand(){
    return new KyleSabatogeJavaPath(robotDrive);
  }
  public Command getDrivePIDTuningCommand(){
    return new DrivePIDTuning(robotDrive);
  }

  public Command getSubsystemEncoderTuningCommand() {
    return new SubsystemEncoderTuning(shoulder, slider, wrist);
  }
  public Command getPIDTuningTestPath(){
    return new PIDTuningTestPaths(robotDrive, (int)SmartDashboard.getNumber("PIDTuningTestPathSelected", 3));
  }

  public Command getPathPlannerLoaderCommand(String pathFileName){
    return new PathPlannerLoader(robotDrive, pathFileName);
  }
  public Command getAutoRampClimbCommand(){
    return new AutoRampClimb(robotDrive);
  }
  public Command getAutoRampClimbPlusMidCubeCommand(){
    return new AutoRampClimbPlusMidCubeCommand(robotDrive, shoulder, wrist, slider, intake);
  }
  public Command getAutoRampExitCommand(){
    return new AutoRampClimbExitCommand(robotDrive);
  }
  public Command getMidConeCommand(){
    return new MidConeCommand(wrist, shoulder, slider);
  }
  public Command getResetCommand(){
    return new ResetSubsystemsCommand(wrist, shoulder, slider);
  }
  public Command getOuttakeConeCommand(){
    return new IntakeCommand(intake, .5, .5);
  }
  public Command getAutoBlueMidConeCommand(){
    return new AutoBlueMidCone(wrist, shoulder, slider, intake, robotDrive);
  }
  public Command getAutoRampClimbPlusHighCubeCommand(){
    return new AutoRampClimbPlusHighCubeCommand(robotDrive, shoulder, wrist, slider, intake);
  }
}
