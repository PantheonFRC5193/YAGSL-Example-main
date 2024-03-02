package frc.robot.commands;

import  frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj.XboxController;

public class ElevatorCommand extends Command {

    private Elevator m_elevator;

    public ElevatorCommand( Elevator elevator){

        m_elevator = elevator;
       

        addRequirements(elevator);

    }
    
     // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {


    return false;
  }

}
