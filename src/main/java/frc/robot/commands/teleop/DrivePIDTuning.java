package frc.robot.commands.teleop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

public class DrivePIDTuning extends CommandBase{
    
  // Field for DriveSubsystem
  private final DriveSubsystem m_robotDrive;
  private PIDController thetaController = new PIDController(1, 0, 0);
  private PIDController xController = new PIDController(1, 0, 0);
  private PIDController yController = new PIDController(1, 0, 0);
  Pose2d target = null;


  double[] pidValues = {Constants.DriveConstants.thetaP, Constants.DriveConstants.thetaI, Constants.DriveConstants.thetaD, Constants.DriveConstants.xP, Constants.DriveConstants.xI, Constants.DriveConstants.xD, Constants.DriveConstants.yP, Constants.DriveConstants.yI, Constants.DriveConstants.yD};
  String [] pidNames = {"theta P", "theta I", "theta D", "xP", "xI", "xD", "yP", "yI", "yD"};

  /** Creates a new DriveCommand. */
  public DrivePIDTuning(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_robotDrive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for(int i = 0; i < pidValues.length; i++){
      SmartDashboard.putNumber(pidNames[i], pidValues[i]);
    }
    target = new Pose2d(new Translation2d(0,10), new Rotation2d(0));
    thetaController.setSetpoint(target.getRotation().getRadians());
    xController.setSetpoint(target.getX());
    yController.setSetpoint(target.getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_robotDrive.setModuleStates(Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(
        xController.calculate(m_robotDrive.getPose().getX(), target.getX()),
        yController.calculate(m_robotDrive.getPose().getY(), target.getY()),
        thetaController.calculate(m_robotDrive.getPose().getRotation().getRadians(), target.getRotation().getRadians()),
        m_robotDrive.getPose().getRotation()
        ))
        );
    //read new PID values
    // for(int i = 0; i < pidValues.length; i++){
    //   pidValues[i] = SmartDashboard.getNumber(pidNames[i], pidValues[i]);
    // }
    // thetaController.setP(pidValues[0]);
    // thetaController.setI(pidValues[1]);
    // thetaController.setD(pidValues[2]);
    // xController.setP(pidValues[3]);
    // xController.setI(pidValues[4]);
    // xController.setD(pidValues[5]);
    // yController.setP(pidValues[6]);
    // yController.setI(pidValues[7]);
    // yController.setD(pidValues[8]);
    if(Robot.getDriveControlJoystick().getYButtonPressed()){
        target = m_robotDrive.getPose().plus(new Transform2d(new Translation2d(0,10), new Rotation2d(0)));
      }


    //   //dirve backward 10
      if(Robot.getDriveControlJoystick().getAButton()){
        target = m_robotDrive.getPose().plus(new Transform2d(new Translation2d(0,-10), new Rotation2d(0)));
      }
    //   //drive right 10
      if(Robot.getDriveControlJoystick().getBButton()){
        target = m_robotDrive.getPose().plus(new Transform2d(new Translation2d(10,0), new Rotation2d(0)));
      }
    //   //drive left 10
      if(Robot.getDriveControlJoystick().getXButton()){
        target = m_robotDrive.getPose().plus(new Transform2d(new Translation2d(-10,0), new Rotation2d(0)));
      }

  }



  


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Todo: Set motors to stop
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
