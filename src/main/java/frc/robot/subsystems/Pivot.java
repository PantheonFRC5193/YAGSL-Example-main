package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
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

    private DigitalInput limitFront;
    private DigitalInput limitBack;

    //Measured values
    //Syntax for encoder values
    //High/Low value for particular state
    //State
    //1 - Intake lowered over front of bot
    //2 - Intake in speaker shooting position
    //3 - Intake in amp scoring position (initial state set by jig)

    private double pivotLow1 = 60;
    private double pivotHigh1 = 56;
    private double pivotLow2 = 32;
    private double pivotHigh2 = 28;
    private double pivotLow3 = 2;
    private double pivotHigh3 = -2;

    private double dt = 2;



    public Pivot()
    {
        motor = new TalonFX(pivotID,"rio");
        pivotConfig = new TalonFXConfiguration();
        limitFront = new DigitalInput(0);
        limitBack = new DigitalInput(2);

        
        System.out.println("Pivot Object Constructed");
        //PhoenixProUtil.checkErrorAndRetry(() -> mTalon.getConfigurator().refresh(shooterConfig));
        //pivotConfig.CurrentLimits.StatorCurrentLimit = 40;
        //pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        //pivotConfig.CurrentLimits.SupplyCurrentLimit = 40;
        //pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        //pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        //pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        //pivotConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
        //pivotConfig.HardwareLimitSwitch.ReverseLimitEnable = false;
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

    public Boolean getFrontLimit(){

        return limitFront.get();

    }

    public Boolean getBackLimit(){

        return limitBack.get();
    }

    public void stop(){
        motor.stopMotor();
    }

    public TalonFXConfiguration getConfiguration(){

        return pivotConfig;
    }

//Move towards rear of robot

    public Command pivotBack(){
        return runOnce(() -> {if(getBackLimit()) {

                                double initialPos = getPosition();
                                double targetPos;
                            
                                //Set target state

                             if(getPosition()<=pivotLow1 && getPosition() >= pivotHigh1){

                                targetPos = (pivotLow2+pivotHigh2)/2; //Sets target encoder position Average of Upper and Lower limit

                          }else if (getPosition()<=pivotLow2 && getPosition() >= pivotHigh2){

                                targetPos = (pivotLow3+pivotHigh3)/2;

                            }else if (getPosition()<=pivotLow3 && getPosition() >= pivotHigh3){

                                targetPos = (pivotLow3+pivotHigh3)/2; //centers pivot arm if already at position 3
                            }else{

                                targetPos = initialPos;
                            }

                            //set 
                            while(targetPos != initialPos){
                                
                                setPower((targetPos - initialPos)/dt);
                            }

                            motor.set(0);

    }});}

    //Move towards front of robot

    public Command pivotForward(){

      return runOnce(() ->  {if(getFrontLimit()){
                double initialPos = getPosition();
                double targetPos;

            //Movement towards front of robot
                if(getPosition()<=pivotLow3 && getPosition() >= pivotHigh3){
    
                    targetPos = (pivotLow2+pivotHigh2)/2; //Sets target encoder position Average of Upper and Lower limit
    
                }else if (getPosition()<=pivotLow2 && getPosition() >= pivotHigh2){
    
                    targetPos = (pivotLow1+pivotHigh1)/2;
    
                }else if (getPosition()<=pivotLow1 && getPosition() >= pivotHigh1){
    
                    targetPos = (pivotLow1+pivotHigh1)/2; //centers pivot arm if already at position 1
                }else
                {
                    targetPos = initialPos;
                }
        
                while(targetPos != getPosition()){
                                
                                setPower((targetPos - initialPos)/dt);
                            }

                            setPower(0);}
            });}

    }
