package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

public class TwoButtonTrigger{

    Trigger button1;
    Trigger button2;

    public TwoButtonTrigger(Trigger trigger1,Trigger trigger2) {
        
        button1 = trigger1;
        button2 = trigger2;

    }
    
    public Trigger get(){
        return new Trigger(() -> {return button1.getAsBoolean() && button2.getAsBoolean();});
    }

    


    
}
