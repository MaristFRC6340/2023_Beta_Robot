package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


import frc.robot.Constants;

public class IntakeSubsystem {
    private TalonFX intakeMotor;
    private double curPower;


    public IntakeSubsystem(){
        intakeMotor = new TalonFX(5);//intake motor is a talon
    }
    
    /**
     * Sets the intake to run at the designated power.
     * @param power
     */
    public void setPower(double power) {
        intakeMotor.set(TalonFXControlMode.PercentOutput, power);
        curPower = power;
    }
    
    public double getPower(){
        return curPower;
    }

    
}
