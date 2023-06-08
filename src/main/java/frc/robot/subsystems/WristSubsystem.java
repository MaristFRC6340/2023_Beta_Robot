package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class WristSubsystem {

    
    private CANSparkMax wristMotor;
    private double encoderCounts;
    private SparkMaxPIDController pidController;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

    public WristSubsystem(){
        wristMotor = new CANSparkMax(8, MotorType.kBrushless);
    }
}

