package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import  frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.signals.NeutralModeValue;



public class PivotCommandUp extends Command {

    Pivot m_pivot;    

    //Preset constants for pivot position
    //Assuming 0 is the initial position at the start of the match
    private double pivotLow1 = 60;
    private double pivotHigh1 = 56;
    private double pivotLow2 = 32;
    private double pivotHigh2 = 28;
    private double pivotLow3 = 2;
    private double pivotHigh3 = -2;

    //
    private double initialPosition;
    
    public PivotCommandUp(Pivot pivot){
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

    //Movement towards front of robot
   if(m_pivot.getPosition()<=pivotLow1 && m_pivot.getPosition() >= pivotHigh1){

                m_pivot.setPosition((pivotLow2+pivotHigh2)/2); //Sets target encoder position Average of Upper and Lower limit

            }else if (m_pivot.getPosition()<=pivotLow2 && m_pivot.getPosition() >= pivotHigh2){

                m_pivot.setPosition((pivotLow3+pivotHigh3)/2);

            }else if (m_pivot.getPosition()<=pivotLow3 && m_pivot.getPosition() >= pivotHigh3){

                m_pivot.setPosition((pivotLow3+pivotHigh3)/2); //centers pivot arm if already at position 3
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
    }else{
        return false;
    }
}
        
    
}
