package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


//Subsystem for Pivot Arm run by Falcon motor with integrated 
//TalonFX motor controller
//

public class Pivot extends SubsystemBase {
    
    private int pivotID = 12;
    private TalonFX motor;
    private TalonFXConfiguration pivotConfig;

    private DigitalInput limit1;
    private DigitalInput limit2;

    public Pivot()
    {
        motor = new TalonFX(pivotID,"rio");
        pivotConfig = new TalonFXConfiguration();
        limit1 = new DigitalInput(2);
        limit2 = new DigitalInput(0);

        
        System.out.println("Pivot Object Constructed");
        //PhoenixProUtil.checkErrorAndRetry(() -> mTalon.getConfigurator().refresh(shooterConfig));
        pivotConfig.CurrentLimits.StatorCurrentLimit = 40;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = 40;
        pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
        pivotConfig.HardwareLimitSwitch.ReverseLimitEnable = false;
       // TalonUtil.applyAndCheckConfiguration(motor, shooterConfig);
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

    public Boolean getLimit1(){

        return limit1.get();

    }

    public Boolean getLimit2(){

        return limit2.get();
    }

    public void stop(){
        motor.stopMotor();
    }

    public TalonFXConfiguration getConfiguration(){

        return pivotConfig;
    }




}