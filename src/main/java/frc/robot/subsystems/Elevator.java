package frc.robot.subsystems;
import com.revrobotics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;


//Thriftybot Elevator Subsystem operated by a Neo Motor
//With a SparkMax Controller.
//Also implements touch limit sensor

public class Elevator extends SubsystemBase {
    //Sparkmax Constants
    private int elevatorID = 18;
    private CANSparkMax motor;
    private RelativeEncoder encoder;

    //Touch sensor
    private DigitalInput limit;


    public Elevator()
    {
        motor = new CANSparkMax(elevatorID,CANSparkLowLevel.MotorType.kBrushless);
        limit = new DigitalInput(1);
        encoder = motor.getEncoder();
    }
    
    public void setPower(double power){

        motor.set(power);
    }

    public double getVelocity(){

        return encoder.getVelocity();
    }
    
    public double getPosition(){
        return encoder.getPosition();

    }

    public void setPosition(double newPosition){

        encoder.setPosition(newPosition);
    }

    public void stop(){
        motor.disable();
    }

    public boolean getLimit(){
        return limit.get();
    }
    
}
