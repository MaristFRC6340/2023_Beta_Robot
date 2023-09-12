// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class RampClimbNavX extends CommandBase {

  private DriveSubsystem  m_drive;
  private int frameCount = 0;
  private double motorPower;
  private double powerMultiplier = 2.75; // USED TO BE 2.75 can be raised for real ramp if wanted
  private int levelCount = 0;
  // Need to get instance or connection to NavX

  /** Creates a new RampClimbNavX. */
  public RampClimbNavX(DriveSubsystem drive) {

    m_drive = drive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO Implement Ramp Balance
    double theta = m_drive.getRoll();
    frameCount++;
    theta+=0.125;
    if(Math.abs(theta)<3){
      levelCount++;
    } else {
      levelCount=0;
    }

    if(theta>0){
      motorPower = Math.sin(Math.toRadians(theta-90))+1;
      motorPower *= powerMultiplier;
    } else {
      motorPower = -Math.sin(Math.toRadians(theta-90))-1;
      motorPower *= powerMultiplier;
    }
    

    double cap = 0.15;
    if(motorPower>cap){
      motorPower=cap;
    }
    if(motorPower<-cap){
      motorPower=-cap;
    }

      m_drive.drive(motorPower, 0, 0, false);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false);
    m_drive.setX();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return levelCount>100;
    // Think about using some time of timout and measure if 
    // near zero for long period
  }
}