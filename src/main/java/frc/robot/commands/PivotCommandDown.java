package frc.robot.commands;

import  frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj.XboxController;


public class PivotCommandDown extends Command {

    private Pivot m_pivot;
    private CommandXboxController m_controller;
    
    //Constants
    //Encoder upper and lower limits for pivot
    //Each band is +/-2 from a center encoder value
    //0 encoder value is the initial position set by a jig 
    //Motion towards the front of the robot is positive
    //Nomenclature coded by low and high limits for each stage
    //1-Intake
    //2-Speaker Shooter
    //3-Amp Scoring/Neutral Position

    private double pivotLow1 = 60;
    private double pivotHigh1 = 56;
    private double pivotLow2 = 32;
    private double pivotHigh2 = 28;
    private double pivotLow3 = 2;
    private double pivotHigh3 = -2;

    private double initialPosition;

    public PivotCommandDown(Pivot pivot){
        m_pivot = pivot;
        initialPosition = m_pivot.getPosition();
        System.out.println("Pivot Object Constructed");
        addRequirements(pivot);
    }


    @Override
    public void initialize(){ 
        initialPosition = m_pivot.getPosition();
    }

    @Override
    public void execute(){

     if(!m_pivot.getLimit1() && !m_pivot.getLimit2()) {
    
        //Movement towards back of robot
        

            

        //Movement towards front of robot
        if(m_controller.leftBumper().getAsBoolean()){

            if(m_pivot.getPosition()<=pivotLow3 && m_pivot.getPosition() >= pivotHigh3){

                m_pivot.setPosition((pivotLow2+pivotHigh2)/2); //Sets target encoder position Average of Upper and Lower limit

            }else if (m_pivot.getPosition()<=pivotLow2 && m_pivot.getPosition() >= pivotHigh2){

                m_pivot.setPosition((pivotLow1+pivotHigh1)/2);

            }else if (m_pivot.getPosition()<=pivotLow1 && m_pivot.getPosition() >= pivotHigh1){

                m_pivot.setPosition((pivotLow1+pivotHigh1)/2); //centers pivot arm if already at position 1
            }

        }

    }

    }

    @Override
    public void end(boolean interrupted){
        m_pivot.stop();
    }

    @Override
    public boolean isFinished(){
        if(m_pivot.getPosition() != initialPosition){
            return true;
        }
        else{
        return false;
        }

    }

}

