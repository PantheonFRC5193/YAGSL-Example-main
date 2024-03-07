// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

//import edu.wpi.first.wpilibj2.command.button.*;
//import edu.wpi.first.wpilibj2.command.button.Trigger;

//Limelight data-value imports and SmartDashboard

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/falcon"));

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController controller = new CommandXboxController(0);

   private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    private final Elevator elevator = new Elevator();
    private final Pivot pivot = new Pivot();
    private final Limelight limelight = new Limelight();


    //private final Command intakeCommand;
    //private final Command pivotCommandU;
    //private final Command pivotCommandD;
    //private final Command shootCommand;
    //private final Command elevatorCommand;


    //Limelight Vision Tracking objects
    //private final NetworkTable networkTableInstance = new NetworkTable( );

    BooleanSupplier aSupplier = () -> controller.a().getAsBoolean();
    BooleanSupplier bSupplier = () -> controller.b().getAsBoolean();
    BooleanSupplier xSupplier = () -> controller.x().getAsBoolean();
    BooleanSupplier ySupplier = () -> controller.y().getAsBoolean();

    //private SequentialCommandGroup pivotForward;
    //private SequentialCommandGroup pivotBack;

    //private ParallelDeadlineGroup pivotForward;
    //private ParallelDeadlineGroup pivotBack;

    private SequentialCommandGroup shootForwardProgressive;
    SendableChooser<Command> autoChooser = new SendableChooser<>(); // auton: path-plannar


    //DoubleSupplier leftYSupplier = () -> -MathUtil.applyDeadband(controller.getHID().getLeftY(),controller.getHID().LEFT_Y_DEADBAND)
    
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {

    autoChooser = AutoBuilder.buildAutoChooser("New Auto"); // auton: path-plannar


 
    //Instantiate command objects
    //intakeCommand = new IntakeCommand(intake,0.0);
    System.out.println("Intake Command Initialized");
    //pivotCommandU = new PivotCommandUp(pivot);
    //pivotCommandD = new PivotCommandDown(pivot);
    //shootCommand = new ShooterCommand(shooter,0.0);
    //elevatorCommand = new ElevatorCommand(elevator);

    //Instantiate Limelight Vision


    //Swerve Drive configurations

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> -MathUtil.applyDeadband(controller.getHID().getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(controller.getHID().getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(controller.getHID().getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   ySupplier,
                                                                   aSupplier,
                                                                   xSupplier,
                                                                   bSupplier);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(controller.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(controller.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> controller.getRightX(),
        () -> controller.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(controller.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(controller.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> controller.getRightX() * 0.5);

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(controller.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(controller.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> controller.getRawAxis(2));

    //Intake and shooter commmands


    Command shootForward = shooter.shootForward(() -> controller.getRightTriggerAxis());
    //Command shootForwardHalf = shooter.shootForward();
    //Command shootBackward = shooter.shootForward(()-> -1*controller.getRightTriggerAxis());

    Command intakeForward = intake.intoBot(() -> controller.getLeftTriggerAxis());//.until(()->{return !intake.getLight();}),intake.setPositionDiff(-1));
    
    //Command intakeBackward = intake.intoBot(() -> -1*controller.getLeftTriggerAxis());
    

    //SequentialCommandGroup shootForwardProgressive = new SequentialCommandGroup(shooter.shootForward(()->controller.getRightTriggerAxis()*0.25),shooter.shootForward(()->controller.getRightTriggerAxis()*0.5),shooter.shootForward(()->controller.getRightTriggerAxis()*0.75),shooter.shootForward(()->controller.getRightTriggerAxis()));


    //Command runPivot = pivot.test();

    //Shooter & Intake Default commands

    shooter.setDefaultCommand(shootForward);
    intake.setDefaultCommand(intakeForward);
    //shooter.setDefaultCommand(shootForwardProgressive);



    //pivotForward = new SequentialCommandGroup(pivot.pivotForward(),pivot.setPowerCommand(0.0));
    //pivotBack = new SequentialCommandGroup(pivot.pivotBack(),pivot.setPowerCommand(0.0));

    //pivotForward = new ParallelDeadlineGroup(pivot.pivotForward(),pivot.checkLimitSwitches());
    //pivotBack = new ParallelDeadlineGroup(pivot.pivotBack(),pivot.checkLimitSwitches());



    //pivot.setDefaultCommand(pivot.checkLimitSwitches());

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);

    // Configure the trigger bindings
        configureBindings();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    //Button commands for driving options

    controller.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    controller.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    controller.b().whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              ));
    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

    //Bumper bindings for pivot
    //controller.rightBumper().onTrue(Commands.runOnce(()->new PivotCommandUp(pivot),pivot));
    //controller.leftBumper().onTrue(Commands.runOnce(()->new PivotCommandDown(pivot),pivot));

    //Directional-pad bindings for pivot that run when both limit switches are not switched

    //controller.x().toggleOnTrue(pivot.pivotForward());
    //controller.b().toggleOnTrue(pivot.pivotBack());

    // Pivot Forward
    //controller.povLeft().onTrue(pivot.setPositionCommand(0.1).until(() -> pivot.isAtPosition()));
    controller.rightBumper().and(()->{return pivot.getBackLimit();}).whileTrue(pivot.pivotBack()).onFalse(pivot.setPowerCommand(0));

    // Pivot Backward 
    controller.leftBumper().and(()->{return pivot.getFrontLimit();}).whileTrue(pivot.pivotForward()).onFalse(pivot.setPowerCommand(0));
    
    //Pivot limits

    //pivot.backLimitTrue().onFalse(Commands.run(()->{pivot.setPower(0);}));
   //pivot.frontLimitTrue().onFalse(Commands.run(()->{pivot.setPower(0);}));


    //Instant reverse when clicking left-stick

   controller.leftStick().onTrue(intake.immediateReverse()).onFalse(intake.setPowerCommand(0));


   //Pivot state positions


   //Move to neutral/start
   //controller.povLeft().and(()->{return pivot.getBackLimit();}).onTrue(pivot.moveToNeutral()).onFalse(pivot.setPowerCommand(0));

   //Move to shooting
  //controller.povUp().onTrue(pivot.moveToShoot()).onFalse(pivot.setPowerCommand(0)).onFalse(pivot.setPowerCommand(0));

    //Move to ground
   //controller.povRight().and(()->{return pivot.getFrontLimit();}).onTrue(pivot.moveToGround()).onFalse(pivot.setPowerCommand(0));

    //controller.

    // SERVO COMMAND __
  controller.rightStick().onTrue(limelight.servoSwitch());

  
  
    //controller.povRight().onTrue(pivot.setPositionCommand(0.1).until(() -> pivot.isAtPosition()));

 
    //Trigger buttons bound to intake and shooter forward motion when bumpers are not pressed

    //controller.leftTrigger(0.3).and(()->{return !controller.leftBumper().getAsBoolean();}).whileTrue(Commands.run(()->intake.setPower(controller.getLeftTriggerAxis()),intake));
    //controller.rightTrigger(0.3).and(()->{return !controller.rightBumper().getAsBoolean();}).whileTrue(Commands.run(()->shooter.setPower(controller.getRightTriggerAxis()),shooter));
   

    //Defining commands within appropriate subsystem (modelled on SwerveSubsystem from YAGSL)

    //controller.leftTrigger(0.1).and(()->{return !controller.leftBumper().getAsBoolean();}).whileTrue(shooter.shootForward(()->{return controller.getLeftTriggerAxis();}));

    //Triggers reverse shooter and intake while corresponding bumpers are pressed
    //Need to come up with a way for motors to not fight each other

    //Two Button Trigger RB and RT
    //For reversing the shooter

  //  TwoButtonTrigger reverseToggleShooter = new TwoButtonTrigger(controller.rightBumper(),controller.rightTrigger(0.1));

    // reverseToggleShooter.get().whileTrue(shooter.shootForward(() -> -1*controller.getRightTriggerAxis() ));


    //Two button Trigger LB and LT
    //For reversing the intake

    //TwoButtonTrigger reverseToggleIntake = new TwoButtonTrigger(controller.leftBumper(),controller.rightTrigger()).
    
    //reverseToggleIntake.get().whileTrue(intake.intoBot(() -> -1*controller.getLeftTriggerAxis()));
                                                

    //y button activates the elevator to a fixed number of rotations based on game requirements
    //Measure encoder values for precise positioning

    //controller.y().onTrue(Commands.runOnce(()->{elevator.setPosition(5);}));

    //Vision readings
    //Using limelight to modify ChassisSpeed of the SwerveSubsystem object constructed here
    //Clicking right stick provides a toggle for a targeting system on the nearest April Tag

    //controller.rightStick().toggleOnTrue( )
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
 

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

public SwerveSubsystem getSwerveSystem(){

    return drivebase;
}

public CommandXboxController getController(){

    return controller;
}

// Auton stuff for Path Plannar
public Command getAutonomousCommand() {
    return autoChooser.getSelected();
}

// Auton stuff for Path Plannar
public Command BlueAuton(){
    return new PathPlannerAuto("Sampled Auton");
}

public Subsystem getIntakeSystem()
{
    return intake;
}

public Subsystem getPivotSystem(){

    return pivot;
}

public Subsystem getShooterSystem(){

    return shooter;
}


//Methods for retrieving stick values on XboxController
//And for returning angle heading
//getLeft(X/Y)D returns a double
//getLeft(X/Y) is a DoubleSupplier interface

private double getLeftXD(){

    if(Math.abs(controller.getLeftX()) < 0.1)
        return 0;
    else
    return controller.getLeftX();
}


public DoubleSupplier getLeftX(){

    DoubleSupplier value = () -> getLeftXD();
    
    return value;
}

 private double getLeftYD(){

    if(Math.abs(controller.getLeftY()) < 0.1)
        return 0;
    else
    return controller.getLeftY();
}

public DoubleSupplier getLeftY()
{

    DoubleSupplier value = () -> getLeftYD();
    
    return value;
}

public DoubleSupplier getAngle(){

    DoubleSupplier m_angle = () -> stickHeading();

       return m_angle;

}

//Stick heading calculation and correction back to [0,2pi)

public double stickHeading(){

    double angleToken = 0;
        
        //Trigonometry using inverse sine and angle corrections based on stick directions to calculate angle heading
        if (getLeftXD() > 0 ){ 
             //Quadrant 1
            if (getLeftYD() > 0){

                angleToken = Math.asin(getLeftYD()/getLeftXD());
                //Quadrant 4
            }else if(getLeftYD()<0){

                angleToken = Math.asin(getLeftYD()/getLeftXD())+2*Math.PI;

            }
    
        } else if(getLeftXD() < 0){ 
                //Quadrant 2
               if (getLeftYD() > 0){
                angleToken = Math.asin(getLeftYD()/getLeftXD())+Math.PI;
                //Quadrant 3
            }else if(getLeftYD()<0){

                angleToken = Math.asin(getLeftYD()/getLeftXD())+Math.PI;

            }

        }else if(getLeftXD()==0 && getLeftYD() >0)
        {
            angleToken =  Math.PI/2;
        }else if (getLeftXD()==0 && getLeftYD() < 0)
        {
            return -1*Math.PI/2;
        }else if (getLeftXD() > 0 && getLeftYD()==0){
            angleToken = 0;
        }else if(getLeftXD() < 0 && getLeftYD() == 0){
            angleToken = Math.PI;
        }else{
            angleToken = 0;
        }
        return angleToken;
       }
}

