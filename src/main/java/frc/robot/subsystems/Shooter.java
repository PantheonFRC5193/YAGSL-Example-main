package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;

import com.ctre.phoenix6.hardware.*;
//import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.configs.*;
//import com.ctre.phoenix.motorcontrol.*;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.*;
//import java.util.function.*;
import java.util.function.DoubleSupplier;

//Subsystem for Shooter, a Falcon motor run by
//integrated TalonFX motor controller

public class Shooter extends SubsystemBase {

    private int shooterID = 29;
    private final TalonFX motor; 
    private final TalonFXConfiguration m_Config;

    public Shooter(){
        
        motor = new TalonFX(shooterID,"rio");

        m_Config = new TalonFXConfiguration();
        //EXAMPLE Set Up for Cheezy Poofs
        //PhoenixProUtil.checkErrorAndRetry(() -> mTalon.getConfigurator().refresh(shooterConfig));
        //shooterConfig.CurrentLimits.StatorCurrentLimit = 40;
        //shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        //m_Config.CurrentLimits.SupplyCurrentLimit = 40;
        //shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_Config.Voltage.PeakForwardVoltage = 12;
        m_Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        //shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        //shooterConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
        //shooterConfig.HardwareLimitSwitch.ReverseLimitEnable = false;
       // TalonUtil.applyAndCheckConfiguration(motor, shooterConfig);

       System.out.println("Shooter Constructed!");

    }

    public void setPower(double power){
        
        motor.set(power);
    }

    public double getVelocity(){
        return motor.getVelocity().getValue();
    }

    public void setPosition(double position){
        motor.setPosition(position);
    }


    public double getPosition(){
       return motor.getPosition().getValue();
    }

    public void stop(){
        motor.stopMotor();
    }

    public void setNeutralMode(){
        m_Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }

    public TalonFXConfiguration getConfiguration(){

        return m_Config;
    }

    public Command shootForward(DoubleSupplier power){
        return run(()-> {setPower(power.getAsDouble()); });
    }

    //public Command shootForward(){

       // return run(()->{setPower(0.5);});
    //}

    //Remap triggers to new high and low values for progressive

    public DoubleSupplier remapTriggerAxis(DoubleSupplier input,double nLow, double nHigh){

        DoubleSupplier nVal = () -> nHigh*input.getAsDouble() + (1-input.getAsDouble())*nLow;
        return nVal;
    }

    
}
