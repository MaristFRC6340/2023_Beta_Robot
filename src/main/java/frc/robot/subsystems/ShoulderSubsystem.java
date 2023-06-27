package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShoulderSubsystem extends SubsystemBase {
    
    
    private CANSparkMax rightShoulderMotor;//Leader Motor
    private CANSparkMax leftShoulderMotor;
    //Only the Leader Actuator needs a PID Controller as the followers will mimic the leaders behaviour
    private SparkMaxPIDController leaderPIDController;
    private RelativeEncoder rightShoulderRelativeEncoder;
    private RelativeEncoder leftShoulderRelativeEncoder;



    public ShoulderSubsystem(){
        //Instantiate the CAN Spark Maxes
        leftShoulderMotor=new CANSparkMax(3, MotorType.kBrushless);
        rightShoulderMotor = new CANSparkMax(4, MotorType.kBrushless);

        //instantiate the leader's (right Shoulder's) PID Controller
        leaderPIDController = rightShoulderMotor.getPIDController();

        //Set PID Controller Variables
        //setting pid variables from constants class
        leaderPIDController.setP(Constants.ShoulderConstants.kP);
        leaderPIDController.setI(Constants.ShoulderConstants.kI);
        leaderPIDController.setD(Constants.ShoulderConstants.kD);
        leaderPIDController.setIZone(Constants.ShoulderConstants.kIz);
        leaderPIDController.setFF(Constants.ShoulderConstants.kFF);
        leaderPIDController.setOutputRange(Constants.ShoulderConstants.kMinOutput, Constants.ShoulderConstants.kMaxOutput);
        
        //instaniate the Relative Encodoer for the Shoulder Motors
        leftShoulderRelativeEncoder = leftShoulderMotor.getEncoder();
        rightShoulderRelativeEncoder = rightShoulderMotor.getEncoder();

        //set the leftShoulder Motor to follow the leader(right) motor
        leftShoulderMotor.follow(rightShoulderMotor);
    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("leftShoulderEncoderPos", leftShoulderRelativeEncoder.getPosition());
        SmartDashboard.putNumber("rightShoulderEncoderPos", rightShoulderRelativeEncoder.getPosition());

    }
    /**
     * run the Shoulder motors to the specified encoder counts and hold it there
     * 
     * @param encoderCounts target encoder counts for the motor to run to
     */
    public void goToPos(double encoderCounts){
        leaderPIDController.setReference(encoderCounts, CANSparkMax.ControlType.kPosition);
    }

    /**
     * 
     */
    public double getPosition(){
        return leftShoulderRelativeEncoder.getPosition();
    }

}
