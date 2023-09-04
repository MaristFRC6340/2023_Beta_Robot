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
    private CANSparkMax leftShoulderMotor;//Follower Motor

    //Only the Leader Actuator needs a PID Controller as the followers will mimic the leaders behaviour
    private SparkMaxPIDController leaderPIDController;
    private RelativeEncoder rightShoulderRelativeEncoder;
    private RelativeEncoder leftShoulderRelativeEncoder;



    public ShoulderSubsystem(){
        //Instantiate the CAN Spark Maxes
        leftShoulderMotor=new CANSparkMax(7, MotorType.kBrushless);
        rightShoulderMotor = new CANSparkMax(6, MotorType.kBrushless);


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

        //invert the right Shoulder motor so raising the arm increases encoder counts
        rightShoulderMotor.setInverted(true);
        rightShoulderMotor.burnFlash();

        //set the leftShoulder Motor to follow the leader(right) motor
        leftShoulderMotor.follow(rightShoulderMotor, true);
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
     * run the shoulder motors at the specified pwoer
     * @param power
     */
    public void setPower(double power){
        //leftShoulderMotor.set(power);
        rightShoulderMotor.set(power);
    }
    /**
     * Resets the encoers of the shoulder motors
     */
    public void resetEncoder(){
        leftShoulderRelativeEncoder.setPosition(0);
        rightShoulderRelativeEncoder.setPosition(0);
       
    }

    /**
     * Return the encoder position of the left shoulder motor; We onlu retrun the left Shoulder Motor encoder 
     * because it is the leader and the right shoulder motor will just follow anything the left motor will do regardless of encoder counts
     */
    public double getPosition(){
        return leftShoulderRelativeEncoder.getPosition();
    }

}
