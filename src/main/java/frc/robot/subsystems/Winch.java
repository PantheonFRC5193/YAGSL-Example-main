package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.configs.*;
//import com.ctre.phoenix.motorcontrol.*;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.*;
//import java.util.function.*;

public class Winch extends SubsystemBase {

    private final int winchID = 37;
    private final TalonFX motor;
    private final TalonFXConfiguration m_Config;

    public Winch(){
        motor = new TalonFX(winchID,"rio");
        m_Config = new TalonFXConfiguration();

        
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

    public void setNeutralMode(){
        m_Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }

    public TalonFXConfiguration getConfiguration(){

        return m_Config;
    }



    
}
