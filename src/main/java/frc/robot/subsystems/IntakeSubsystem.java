package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


import frc.robot.Constants;

public class IntakeSubsystem {
    private TalonFX intakeMotor;
    private double curPower;


    public IntakeSubsystem(){
        intakeMotor = new TalonFX(5);
    }

    public void intake(){
        intakeMotor.set(TalonFXControlMode.PercentOutput, Constants.IntakeConstants.motorPower);
        curPower = Constants.IntakeConstants.motorPower;
    }
    public void outtake(){
        intakeMotor.set(TalonFXControlMode.PercentOutput,-Constants.IntakeConstants.motorPower);
        curPower = -Constants.IntakeConstants.motorPower;

    }

    public void setPower(double power) {
        intakeMotor.set(TalonFXControlMode.PercentOutput, power);
        curPower = power;
    }
    public void stop(){
        intakeMotor.set(TalonFXControlMode.PercentOutput,0);
        curPower = 0;
    }
    public double getPower(){
        return curPower;
    }

    
}
