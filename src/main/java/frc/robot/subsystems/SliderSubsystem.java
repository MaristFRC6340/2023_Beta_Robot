package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SliderSubsystem extends SubsystemBase {
    
    
    private CANSparkMax rightSliderMotor;//Leader Motor
    private CANSparkMax leftSliderMotor;//follower 

    //Only the Leader Actuator needs a PID Controller as the followers will mimic the leaders behaviour
     private SparkMaxPIDController leaderPIDController;
     private RelativeEncoder rightSliderRelativeEncoder;
    private RelativeEncoder leftSliderRelativeEncoder;

    //If you set one motor to follow another one it will mimic any power sent to the leader motor
    //This is handy for our SLider subsystem because we have two motors working in tandem and we dont want them to 
    //get out of sync with each other



    public SliderSubsystem(){
        //Instantiate the CAN Spark Maxes
        leftSliderMotor=new CANSparkMax(4, MotorType.kBrushless); 
        rightSliderMotor = new CANSparkMax(3, MotorType.kBrushless);

        //instantiate the leader's (right slider's) PID Controller
        leaderPIDController = rightSliderMotor.getPIDController();

        //Set PID Controller Variables
        //setting pid variables from constants class
        leaderPIDController.setP(Constants.SliderConstants.kP);
        leaderPIDController.setI(Constants.SliderConstants.kI);
        leaderPIDController.setD(Constants.SliderConstants.kD);
        leaderPIDController.setIZone(Constants.SliderConstants.kIz);
        leaderPIDController.setFF(Constants.SliderConstants.kFF);
        leaderPIDController.setOutputRange(Constants.SliderConstants.kMinOutput, Constants.SliderConstants.kMaxOutput);
        
      


        //instaniate the Relative Encodoer for the slider Motors
        leftSliderRelativeEncoder = leftSliderMotor.getEncoder();
        rightSliderRelativeEncoder = rightSliderMotor.getEncoder();

        //set the rightSliderMotor to be INverted
        rightSliderMotor.setInverted(true);
        rightSliderMotor.burnFlash();

        //set the leftSlider Motor to follow the leader(right) motor, the second boolean parameter inverts the follower motor so that
        //it runs exactly opposite of the leader. The sliders have to turn in opposite directions from each other for it to work
       leftSliderMotor.follow(rightSliderMotor, true);


    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("leftSliderEncoderPos", leftSliderRelativeEncoder.getPosition());
        SmartDashboard.putNumber("rightSliderEncoderPos", rightSliderRelativeEncoder.getPosition());

    }
    /**
     * run the slider motors to the specified encoder counts and hold it there
     * 
     * @param encoderCounts target encoder counts for the motor to run to
     */
    public void goToPos(double encoderCounts){
        leaderPIDController.setReference(encoderCounts, CANSparkMax.ControlType.kPosition);
    }
    /**
     * Set the power going to the motors range [-1,1]
     * @param power
     */
    public void setPower(double power){
        rightSliderMotor.set(power);

    }
    /**
     * Reset encoders of the left and right Slider Motors
     */
    public void resetEncoder(){
        leftSliderRelativeEncoder.setPosition(0);
        rightSliderRelativeEncoder.setPosition(0);
    }

    /**
     * Get the encoder position of the slider; We only get the encoder position of the 
     * the right motor because it is the leader motor and he left motors will just mimic whatever the right motor does
     */
    public double getPosition(){
        return rightSliderRelativeEncoder.getPosition();
    }

}
