package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;


import frc.robot.Constants;

public class IntakeSubsystem {
    private CANSparkMax intake;
    private double curPower;


    public IntakeSubsystem(){
        intakeMotor = new CANSparkMax(5, CANSparkMax.MotorType.kBrushless);
    }

    public void intake(){
        intakeMotor.set(Constants.IntakeConstants.motorPower);
        curPower = Constants.IntakeConstants.motorPower;
    }
    public void outtake(){
        intakeMotor.set(-Constants.IntakeConstants.motorPower);
        curPower = -Constants.IntakeConstants.motorPower;

    }
    public void stop(){
        intakeMotor.set(0);
        curPower = 0;
    }
    public double getPower(){
        return curPower;
    }

    
}
