// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.teleop.DriveTeleopCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;


public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);
  private Field2d field = new Field2d();

  

  // The gyro sensor
  // private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
  private final AHRS m_gryo = new AHRS(SPI.Port.kMXP);
  double rotOffset = 90;


  private double driveP = Constants.ModuleConstants.kDrivingP;
  private double driveI = Constants.ModuleConstants.kDrivingI;
  private double driveD = Constants.ModuleConstants.kDrivingD;


  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(-m_gryo.getAngle()), // Compass - make negative
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    SmartDashboard.putNumber("Drive P", driveP);
    SmartDashboard.putNumber("Drive I", driveI);
    SmartDashboard.putNumber("Drive D", driveD);
    SmartDashboard.putData(field);

  }

  int count = 0;

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(-m_gryo.getAngle()), // Reversed for now, make negative
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
        
        SmartDashboard.putNumber("Module 0 azimuth(FRONT LEFT)", m_frontLeft.getAngle());
        SmartDashboard.putNumber("Module 1 azimuth(FRONT_RIGHT", m_frontRight.getAngle());
        SmartDashboard.putNumber("Module 2 azimuth(REAR LEFT)", m_rearLeft.getAngle());
        SmartDashboard.putNumber("Module 3 azimuth (REAR RIGHT)", m_rearRight.getAngle());
        SmartDashboard.putNumber("Gyro", m_gryo.getAngle());
        SmartDashboard.putNumber("Roll: ", m_gryo.getRoll());
        SmartDashboard.putNumber("Orientation", m_odometry.getPoseMeters().getRotation().getDegrees());
        //update robot position on field
      field.setRobotPose(getPose());

      
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gryo.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Adjust input based on max speed
    xSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    ySpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    rot *= DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(-m_gryo.getAngle()-rotOffset))
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }
  public void faceWheelsFront(){
    setModuleStates(new SwerveModuleState[]{new SwerveModuleState(0, Rotation2d.fromDegrees(0)), new SwerveModuleState(0, Rotation2d.fromDegrees(0)), new SwerveModuleState(0, Rotation2d.fromDegrees(0)), new SwerveModuleState(0, Rotation2d.fromDegrees(0))});

  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading(double offset) {
    m_gryo.reset();
    rotOffset = offset;
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gryo.getAngle()).getDegrees()-rotOffset; // changed from - to + 8/14/23
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gryo.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public SequentialCommandGroup followTrajectoryCommand(PathPlannerTrajectory path, boolean isFirstPath){
      PIDController thetaController = new PIDController(Constants.DriveConstants.thetaP,  Constants.DriveConstants.thetaI,  Constants.DriveConstants.thetaD);//-.7
      PIDController xController = new PIDController(Constants.DriveConstants.xP, Constants.DriveConstants.xI, Constants.DriveConstants.xD);
      PIDController yController = new PIDController(Constants.DriveConstants.yP, Constants.DriveConstants.yI, Constants.DriveConstants.yD);

      thetaController.enableContinuousInput(-Math.PI, Math.PI);
      return new SequentialCommandGroup(
        new InstantCommand(() ->{
          if(isFirstPath){this.resetOdometry(path.getInitialHolonomicPose());}
        }),
        new PPSwerveControllerCommand(
          path,
          this::getPose,
          Constants.DriveConstants.kDriveKinematics,
          xController,
          yController,
          thetaController,
          this::setModuleStates,
          true,
          this
        )
        
      );
  }

  public double getRoll(){
    return m_gryo.getRoll();
  }
}

