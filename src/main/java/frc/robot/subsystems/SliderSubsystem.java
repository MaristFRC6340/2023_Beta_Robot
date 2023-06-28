package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SliderSubsystem extends SubsystemBase {
    
    
    //private CANSparkMax rightSliderMotor;//Leader Motor
    private CANSparkMax leftSliderMotor;
    //Only the Leader Actuator needs a PID Controller as the followers will mimic the leaders behaviour
    // private SparkMaxPIDController leaderPIDController;
    private SparkMaxPIDController followerPIDController;
    // private RelativeEncoder rightSliderRelativeEncoder;
    private RelativeEncoder leftSliderRelativeEncoder;



    public SliderSubsystem(){
        //Instantiate the CAN Spark Maxes
        //leftSliderMotor=new CANSparkMax(4, MotorType.kBrushless); // was 3
        //rightSliderMotor = new CANSparkMax(4, MotorType.kBrushless);

        //instantiate the leader's (right slider's) PID Controller
        //leaderPIDController = rightSliderMotor.getPIDController();
        //followerPIDController = leftSliderMotor.getPIDController();

        //Set PID Controller Variables
        //setting pid variables from constants class
        // leaderPIDController.setP(Constants.SliderConstants.kP);
        // leaderPIDController.setI(Constants.SliderConstants.kI);
        // leaderPIDController.setD(Constants.SliderConstants.kD);
        // leaderPIDController.setIZone(Constants.SliderConstants.kIz);
        // leaderPIDController.setFF(Constants.SliderConstants.kFF);
        // leaderPIDController.setOutputRange(Constants.SliderConstants.kMinOutput, Constants.SliderConstants.kMaxOutput);
        
        // followerPIDController.setP(Constants.SliderConstants.kP);
        // followerPIDController.setI(Constants.SliderConstants.kI);
        // followerPIDController.setD(Constants.SliderConstants.kD);
        // followerPIDController.setIZone(Constants.SliderConstants.kIz);
        // followerPIDController.setFF(Constants.SliderConstants.kFF);
        // followerPIDController.setOutputRange(Constants.SliderConstants.kMinOutput, Constants.SliderConstants.kMaxOutput);


        //instaniate the Relative Encodoer for the slider Motors
        //leftSliderRelativeEncoder = leftSliderMotor.getEncoder();
        //rightSliderRelativeEncoder = rightSliderMotor.getEncoder();

        //set the leftSlider Motor to follow the leader(right) motor
        //leftSliderMotor.follow(rightSliderMotor);
    }
    @Override
    public void periodic(){
        //SmartDashboard.putNumber("leftSliderEncoderPos", leftSliderRelativeEncoder.getPosition());
        //SmartDashboard.putNumber("rightSliderEncoderPos", rightSliderRelativeEncoder.getPosition());

    }
    /**
     * run the slider motors to the specified encoder counts and hold it there
     * 
     * @param encoderCounts target encoder counts for the motor to run to
     */
    public void goToPos(double encoderCounts){
        //leaderPIDController.setReference(encoderCounts, CANSparkMax.ControlType.kPosition);
        //followerPIDController.setReference(encoderCounts, CANSparkMax.ControlType.kPosition);
        //followerPIDController.setReference(10, CANSparkMax.ControlType.kPosition);
        //leftSliderMotor.set(0.2);
    }

    public void resetEncoder(){
        //leftSliderRelativeEncoder.setPosition(0);
        //rightSliderRelativeEncoder.setPosition(0);
    }

    /**
     * 
     */
    public double getPosition(){
        //return leftSliderRelativeEncoder.getPosition();
        return 0;
    }

}
