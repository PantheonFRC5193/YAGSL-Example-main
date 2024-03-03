package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.*;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.smartdashboard.*;

public class IntakeCommand extends Command {

    private Intake m_intake;
    private double m_inputPower;


    public IntakeCommand(Intake intake,double inputPower) {
        m_intake = intake;
        m_inputPower = inputPower;
        //System.out.println("Intake Object Constructed");
        addRequirements(intake);
      }
    
      // Called when the command is initially scheduled.
      @Override
      public void initialize() {

        m_intake.setPower(0.5*m_inputPower);

      }
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
       m_intake.setPower(m_inputPower);
      }
    
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
        m_intake.stop();
      }
    
      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        return false;
      }
    
}
