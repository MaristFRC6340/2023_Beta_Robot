// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This is the Beta Robot Code for 2023 Marist FRC Team 6340
// Test Commit 21 Apr

package frc.robot;

import java.util.List;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  //Auto Chooser
  private final SendableChooser<Command> chooser = new SendableChooser<Command>();
  private final SendableChooser<Command[]> teleopChooser = new SendableChooser<Command[]>();

  //Sendable Chooser objects allow you to send a list of items to the Smart Dashboard and then acess whichever one the 
  //user chooses. We use sendable choosers to send a lis of commands. Whichever command the user selects is scheduled



  // TODO: Define the controllers for driving / arm control in static form
  //private static Joystick m_armControlJoystick = new Joystick(Constants.OIConstants.kArmControllerPort); // Port zero for left joystick
  private static XboxController m_driverControlJoystick = new XboxController(Constants.OIConstants.kDriverControllerPort);
  private static XboxController m_armControlJoystick = new XboxController(Constants.OIConstants.kArmControllerPort);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();




    //Read through all the files in the deploy/pathplanner folder and add them as auto options
    ArrayList<String> pathNames = new ArrayList<String>();


    File pathFolder = new File("/home/lvuser/deploy/pathplanner");//acess the path folder on the robot rio. This is where the .path files in the pathplanner folder are put on the robo rio; (I found this by sshing into the robot rio  and digging through the folders)
    try{
      for(File f: pathFolder.listFiles()){//iterate through all the files in the folder
        if(f.isFile() && f.getName().contains(".path")){//if the file is a .path file
          pathNames.add(f.getName().replace(".path", ""));//add it to the Path Names list
        }
      }
    }
    catch(NullPointerException e){
      e.printStackTrace();
    }
    //Add all the found .path file names to the Auto Chooser on Smart Dashboard/ShuffleBoard
    for(String pathFile: pathNames){
      chooser.addOption(pathFile + " (Generated from deploy folder)", m_robotContainer.getPathPlannerLoaderCommand(pathFile));
    }

    //List Predefined Autos
    chooser.setDefaultOption("Default Auto", m_robotContainer.getTestPathCommand());
    chooser.addOption("TestPathPlanner", m_robotContainer.getTestPathCommand());
    chooser.addOption("StraightLineAuto", m_robotContainer.getStraightLineAuto());
    chooser.addOption("KyleSabatogeJava", m_robotContainer.getKyleSabatogeCommand());
    chooser.addOption("PIDTuningTestPaths", m_robotContainer.getPIDTuningTestPath());
    SmartDashboard.putData("Auto List",chooser);

    //List Choosable Teleops
    teleopChooser.setDefaultOption("Default/Competition Teleop", new Command[]{m_robotContainer.getDriveTeleopCommand(), m_robotContainer.getShoulderTeleopCommand(), m_robotContainer.getSliderTeleopCommand(), m_robotContainer.getWristTeleopCommand(), m_robotContainer.getIntakeTeleopCommand()});
    teleopChooser.addOption("Testing Teleop(Encoder and PID Tuning)", new Command[]{m_robotContainer.getDrivePIDTuningCommand(), m_robotContainer.getSubsystemEncoderTuningCommand()});
    SmartDashboard.putData("Teleop List", teleopChooser);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    //Retreive the selected auto command from the Sendable Chooser Object
    m_autonomousCommand = chooser.getSelected();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  
    //Schedule all the teleop commands selected
    for(Command command: teleopChooser.getSelected()){
      command.schedule();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  public static XboxController getArmControlJoystick(){
    return m_armControlJoystick;
  }

  public static XboxController getDriveControlJoystick() {
    return m_driverControlJoystick;
  }

}
