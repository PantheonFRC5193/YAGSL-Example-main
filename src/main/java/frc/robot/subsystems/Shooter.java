package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;

import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.configs.*;
//import com.ctre.phoenix.motorcontrol.*;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.*;
import java.util.function.*;

//Subsystem for Shooter, a Falcon motor run by
//integrated TalonFX motor controller

public class Shooter extends SubsystemBase {

    private int shooterID = 29;
    private final TalonFX motor; 
    private final TalonFXConfiguration shooterConfig;




    public Shooter(){
        
        motor = new TalonFX(shooterID,"rio");

        shooterConfig = new TalonFXConfiguration();
        //EXAMPLE Set Up for Cheezy Poofs
        //PhoenixProUtil.checkErrorAndRetry(() -> mTalon.getConfigurator().refresh(shooterConfig));
        //shooterConfig.CurrentLimits.StatorCurrentLimit = 40;
        //shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        //shooterConfig.CurrentLimits.SupplyCurrentLimit = 40;
        //shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        //shooterConfig.Voltage.PeakForwardVoltage = 12;
        //shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
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
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }

    public TalonFXConfiguration getConfiguration(){

        return shooterConfig;
    }

    public Command shootForward(DoubleSupplier power){
        return run(()-> {setPower(power.getAsDouble()); });
    }

    public Command shootForward(){

        return run(()->{setPower(0.5);});
    }

    
}
