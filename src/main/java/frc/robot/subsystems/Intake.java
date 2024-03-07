package frc.robot.subsystems;
import com.revrobotics.*;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.*;


//import com.revrobotics.CANSparkMax.*;
//import com.revrobotics.CANSparkMax.IdleMode;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Intake extends SubsystemBase {
    
    //SparkMax Motor Controller Info
    private int intakeID = 16;
    private CANSparkMax motor;
    private RelativeEncoder encoder;
    private SparkPIDController m_pidController; 

    //PID Coefficients

    private double kP = 0.1; 
    private double  kI = 1e-4;
    private double  kD = 1; 
    private double  kIz = 0; 
    private double  kFF = 0; 
    private double  kMaxOutput = 1; 
    private double  kMinOutput = -1;


    //Sensors
    //private Port lightPort;
    //private I2C.Port  m_I2Cbus;
    private ColorSensorV3 lightSensor;
    

    public Intake(){
        System.out.println("Intake Object Constructed");
        motor = new CANSparkMax(intakeID,CANSparkLowLevel.MotorType.kBrushless);
       encoder = motor.getEncoder();
       m_pidController =  motor.getPIDController();


        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        //m_I2Cbus = new Port();

       //lightSensor = new ColorSensorV3(lightPort);
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

    public void stop(){
        motor.disable();
    }

    public Command setPowerCommand(double newPower){

        return runOnce(() -> {setPower(newPower); });
    }

    public Command immediateReverse(){

        return runOnce(() ->{double currentPos = encoder.getPosition();
        
            double targetPos = currentPos - 20;

            m_pidController.setReference(targetPos,CANSparkMax.ControlType.kPosition);

        } );
    }

    public Command intoBot(DoubleSupplier power){
        return run(() -> setPower(power.getAsDouble()));

    }

    public boolean getLightRed(){
        return lightSensor.getRed() > 200;
    }


}
