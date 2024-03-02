package frc.robot.commands;

import  frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import edu.wpi.first.wpilibj.XboxController;
public class ShooterCommand extends Command {


    private Shooter m_shooter;
    private double power;

    public ShooterCommand(Shooter shooter, Double inputPower){

        m_shooter = shooter;
        power = inputPower;
        
        addRequirements(shooter);

    }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Shooter command running!");
    m_shooter.getConfiguration().MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_shooter.setPower(0.1*power);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    m_shooter.setPower(power);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_shooter.setPower(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return false;
  }
    
}
