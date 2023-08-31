package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase{

    
    private CANSparkMax wristMotor;
    private SparkMaxPIDController wristPidController;
    private RelativeEncoder wristRelativeEncoder;


    public WristSubsystem(){
        wristMotor = new CANSparkMax(8, MotorType.kBrushless);
        //getting the encoder from CANSparkMax
        wristRelativeEncoder = wristMotor.getEncoder();
        //getting the PID controller from the CANSparkMax
        wristPidController=wristMotor.getPIDController();


        //setting pid variables from constants class
        wristPidController.setP(Constants.WristConstants.kP);
        wristPidController.setI(Constants.WristConstants.kI);
        wristPidController.setD(Constants.WristConstants.kD);
        wristPidController.setIZone(Constants.WristConstants.kIz);
        wristPidController.setFF(Constants.WristConstants.kFF);
        wristPidController.setOutputRange(Constants.WristConstants.kMinOutput, Constants.WristConstants.kMaxOutput);
    }

    /**
     * run the wrist motor to the specified encoder counts and hold it there
     * 
     * @param encoderCounts target encoder counts for the motor to run to
     */
    public void goToPosition (double encoderCounts){
        //set pidControllar position reference( This should make the motor maintain this encoder position :/)
        wristPidController.setReference(encoderCounts, CANSparkMax.ControlType.kPosition);
    }

    /**
     * 
     */
    public void setPower(double pow){
        wristMotor.set(pow);
    }

    /**
     * retrieve the current encoder position of the wrist motor
     * @return encoder position retrieved from relative encoder object
     */
    public double getPosition(){
        return wristRelativeEncoder.getPosition(); 
    }

    public void resetEncoder(){
        wristRelativeEncoder.setPosition(0.0);
    }

    /**
     * This method is called repeatedly by the robot
     * We will use it for telemetry
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Encoder", getPosition());

    }
}

