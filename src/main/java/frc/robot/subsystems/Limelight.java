package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Limelight extends SubsystemBase {


    NetworkTable table;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    Servo limelightServo;    

    double x;
    double y;
    double area;
    double angle;


public Limelight(){



table = NetworkTableInstance.getDefault().getTable("limelight");
tx = table.getEntry("tx");
ty = table.getEntry("ty");
ta = table.getEntry("ta");
limelightServo = new Servo(0); // fix channel for acutal Servo


//read values periodically
x = tx.getDouble(0.0);
y = ty.getDouble(0.0);
area = ta.getDouble(0.0);

angle = limelightServo.getAngle();

System.out.println("Servo Angle " + limelightServo.getAngle());

}



public  Command servoSwitch() {

return runOnce(() -> {if (table.getEntry("getpipe").getDouble(0) == 0) {
    table.getEntry("pipeline").setNumber(1);
    limelightServo.set(0.4
    );

}else if(table.getEntry("getpipe")
.getDouble(0) == 1)  { 
    table.getEntry("pipeline").setNumber(0);
    limelightServo.set(1
    );}
});


}




protected void execute() {
//post to smart dashboard periodically
SmartDashboard.putNumber("LimelightX", x);
SmartDashboard.putNumber("LimelightY", y);
SmartDashboard.putNumber("LimelightArea", area);
SmartDashboard.putNumber("Servo Angle ", angle);
}

}
