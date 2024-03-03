// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import frc.robot.commands.*;
//import edu.wpi.first.wpilibj2.command.button.*;
//import edu.wpi.first.wpilibj2.command.button.Trigger;


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

    private final Command intakeCommand;
    private final Command pivotCommandU;
    private final Command pivotCommandD;
    private final Command shootCommand;
    private final Command elevatorCommand;
    
    //Instantiate command objects
    //private final Command intakeCommand = new IntakeCommand(intake);
    //private final Command pivotCommandU = new PivotCommandUp(pivot);
    //private final Command pivotCommandD = new PivotCommandDown(pivot);
    //private final Command shootCommand = new ShooterCommand(shooter);
    //private final Command elevatorCommand = new ElevatorCommand(elevator);

    BooleanSupplier aSupplier = () -> controller.a().getAsBoolean();
    BooleanSupplier bSupplier = () -> controller.b().getAsBoolean();
    BooleanSupplier xSupplier = () -> controller.x().getAsBoolean();
    BooleanSupplier ySupplier = () -> controller.y().getAsBoolean();



    //DoubleSupplier leftYSupplier = () -> -MathUtil.applyDeadband(controller.getHID().getLeftY(),controller.getHID().LEFT_Y_DEADBAND)
    
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    //Instantiate command objects
    intakeCommand = new IntakeCommand(intake,0.0);
    pivotCommandU = new PivotCommandUp(pivot);
    pivotCommandD = new PivotCommandDown(pivot);
    shootCommand = new ShooterCommand(shooter,0.0);
    elevatorCommand = new ElevatorCommand(elevator);


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

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);

    //Bullhead commands

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

    controller.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    controller.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    controller.b().whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              ));

    //Bumper bindings for pivot
    //controller.rightBumper().onTrue(Commands.runOnce(()->new PivotCommandUp(pivot),pivot));
    //controller.leftBumper().onTrue(Commands.runOnce(()->new PivotCommandDown(pivot),pivot));

    //Directional-pad bindings for pivot that run when both limit switches are not switched

    controller.povLeft()
    .and(()-> {return !pivot.getLimit1();})
    .and(()-> {return !pivot.getLimit2();})
                                            .onTrue(Commands.runOnce(()->new PivotCommandDown(pivot),pivot));

    controller.povRight()
    .and(()-> {return !pivot.getLimit1();})
    .and(()-> {return !pivot.getLimit2();})
                                            .onTrue(Commands.runOnce(()-> new PivotCommandUp(pivot),pivot));


    //Trigger buttons bound to intake and shooter forward motion when bumpers are not pressed

    controller.leftTrigger(0.3)
    .and(()->{return !controller.leftBumper().getAsBoolean();})
                                    .whileTrue(Commands.run(()->shooter.setPower(controller.getLeftTriggerAxis()),shooter));

    controller.rightTrigger(0.3)
    .and(()->{return !controller.rightBumper().getAsBoolean();})
                                    .whileTrue(Commands.run(()->shooter.setPower(controller.getRightTriggerAxis()),shooter));
   

    //Triggers reverse shooter and intake while bumpers are pressed
    controller.rightBumper()
    .and(()->{return controller.getRightTriggerAxis() >=0.3;})
                                            .whileTrue(Commands.run(()->shooter.setPower(-controller.getRightTriggerAxis()),shooter));

    controller.leftBumper()
    .and(()->{return controller.getLeftTriggerAxis() >= 0.3;})
                                            .whileTrue(Commands.run(()->intake.setPower(-controller.getLeftTriggerAxis()),shooter));
                                                

    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());


    //y button activates the elevator to a fixed number of rotations 
    //Measure encoder values for precise positioning

    //controller.y().onTrue(Commands.runOnce(()->{elevator.setPosition(5);}));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }


  public Command getIntakeCommand(){

    return intakeCommand;
}

public Command getPivotCommandUp(){

    return pivotCommandU;
}

public Command getPivotCommandDown(){

    return pivotCommandD;
}

public Command getShooterCommand(){

    return shootCommand;
}

public Command getElevatorCommand(){

    return elevatorCommand;

}

public SwerveSubsystem getSwerveSystem(){

    return drivebase;
}

public CommandXboxController getController(){

    return controller;
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

