package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.configs.*;


//Subsystem for Pivot Arm run by Falcon motor with integrated 
//TalonFX motor controller
//

public class Pivot extends SubsystemBase {
    
    private int pivotID = 12;
    private TalonFX motor;
    private TalonFXConfiguration pivotConfig;

    private DigitalInput limitFront;
    private DigitalInput limitBack;


    //Syntax for encoder values
    //High/Low value for particular state
    //State
    //1 - Intake lowered over front of bot
    //2 - Intake in speaker shooting position
    //3 - Intake in amp scoring position (initial state set by jig)
    //These encoder values have been measured 

    //private double pivotLow1 = 66;
    //private double pivotHigh1 =64;
    //private double pivotLow2 =54 ;
    //private double pivotHigh2 = 56;
    //private double pivotLow3 = 0.5;
    //private double pivotHigh3 = -0.5;
    private double m_position;

    //Target encoder Values

    private double pos0target = 0;
    private double pos1target = 54;
    private double pos2target = 65;

    private Slot0Configs slot0Configs;


    //ternary position variable initialized to 
    //0 is initial pos (set in constructor)
    //1 is shooting position
    //2 is intake position.
    private int positionState; 

    // private double dt = 2;

    public Pivot()
    {
        motor = new TalonFX(pivotID,"rio");
        motor.setPosition(0);
        pivotConfig = new TalonFXConfiguration();
        

        limitBack = new DigitalInput(0);
        limitFront = new DigitalInput(2);

        positionState = 0;

        //PID Constants for slot0 (need tuning 03-04-24 BY)

         slot0Configs = new Slot0Configs();

         slot0Configs.kP = 0.6;
         slot0Configs.kI = 0;
         slot0Configs.kD = 0;

        motor.getConfigurator().apply(slot0Configs);

        System.out.println("Pivot Object Constructed");
        //PhoenixProUtil.checkErrorAndRetry(() -> mTalon.getConfigurator().refresh(shooterConfig));
        //pivotConfig.CurrentLimits.StatorCurrentLimit = 40;
        //pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        //pivotConfig.CurrentLimits.SupplyCurrentLimit = 40;
        //pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
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
        m_position = position;
        motor.setPosition(position);
    }


    public double getPosition(){
       return motor.getPosition().getValue();
    }

    public Boolean getFrontLimit(){
        System.out.println("Front limit reads " + limitFront.get() +"!");
        return limitFront.get();

    }

    public Boolean getBackLimit(){
        //System.out.println("Back limit reads " + limitBack.get() +"!");
        return limitBack.get();
        
    }

    public void stop(){
        motor.stopMotor();
    }

    public TalonFXConfiguration getConfiguration(){

        return pivotConfig;
    }

    //Command for setting power
    public Command setPowerCommand(double newPower){
        return runOnce(() -> {setPower(newPower);});
    }

    public Command checkLimitSwitches(){
       return runOnce(() -> {if(!getBackLimit() || !getFrontLimit()){
                                setPower(0);}
        });
    }

    public boolean isAtPosition() {
      return Math.abs(m_position - motor.getPosition().getValue()) < 1;
    }

    public double getTargetPosition(boolean direction){

        double targetPosition = getPosition();

        //Check if the motor is trying to move forward or back
        //direction true is forward
        //direction false is back
        if(direction){

            if(positionState == 0){

                targetPosition = pos1target;

            }else if(positionState == 1){

                targetPosition = pos2target;
            }
        }else if(!direction){

            if(positionState ==1){

                targetPosition = pos0target;
            }else if(positionState == 2){

                targetPosition = pos1target;
            }

        }

        return targetPosition;

    }

    public Command moveToGround(){

        return runOnce(()->{PositionVoltage request = new PositionVoltage(0).withSlot(0);
    
                            motor.setControl(request.withPosition(64).withVelocity(1));});
    
    }

    public Command moveToShoot(){

       return run(() -> {PositionVoltage request = new PositionVoltage(0).withSlot(0);
    
        motor.setControl(request.withPosition(55).withVelocity(1));});
    
    }
    
    public Command moveToNeutral(){

        return run(()->{PositionVoltage request = new PositionVoltage(0).withSlot(0);
        
        motor.setControl(request.withPosition(0).withVelocity(1));});
        }
    



//Move towards rear of robot
    //public Command pivotBack(){
        //return run(() -> {  double targetPos;
                            
                                //Set target state

                                //targetPos =getTargetPosition(false);

                            //PositionVoltage request = new PositionVoltage(targetPos).withSlot(0);

                            //move into new position
                            //motor.setControl(request.withVelocity(-20));

                            //motor.setControl(request.withPosition(targetPos).withVelocity(0));
    //});}

    //Move towards front of robot

    //public Command pivotForward(){

      //return run(() ->  {
                //double initialPos = getPosition();
                //double targetPos;

                //Identify target position

                //targetPos = getTargetPosition(true);

                //Set a PID request
                  //PositionVoltage request = new PositionVoltage(targetPos).withSlot(0);

                //move into new position
                //motor.setControl(request.withVelocity(20));
                
                 //motor.setControl(request.withPosition(targetPos).withVelocity(0));
                
                //});

                 
                //}

    public Command pivotBack(){
        return run(()->{if(getBackLimit()){setPower(-0.5);}else if(!getBackLimit()) {setPower(0);}}).until(()->{return !getBackLimit();});
    }

    public Command pivotForward(){
        return run(()->{if(getFrontLimit()){setPower(0.1);}else if(!getFrontLimit()){setPower(0);}}).until(()->{return !getFrontLimit();});
    }


    public Trigger backLimitTrue(){
         Trigger backLimitTrigger = new Trigger(limitBack::get);
         return backLimitTrigger;
    }

    public Trigger frontLimitTrue(){
        Trigger frontLimitTrigger = new Trigger(limitFront::get);
        return frontLimitTrigger;
    }

    @Override
    public void periodic() {


    }


    }
