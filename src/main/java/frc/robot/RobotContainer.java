// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExtensionCommand;
import frc.robot.commands.ParallelLinkageTurnCommand;
import frc.robot.commands.ParallelLinkageWristCommand;
// import frc.robot.commands.ShooterLinkageMoverCommand;
import frc.robot.commands.SpringAssemblyShooterCommand;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.ParallelLinkageTurnSubsystem;
// import frc.robot.subsystems.ShooterLinkageMoverSubsystem;
import frc.robot.subsystems.SpringAssemblyShooterSubsystem;
import frc.robot.subsystems.ParallelLinkageWristSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.*;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  //private final IntakeSubsystem intake = new IntakeSubsystem();
  //private final ParallelLinkageWristSubsystem paraLinkage = new ParallelLinkageWristSubsystem(5, -4, 4);
  //private final ParallelLinkageWristCommand linkageCmd = new ParallelLinkageWristCommand(paraLinkage, false);

  /*
  private final ParallelLinkageTurnSubsystem turnSubsystem = new ParallelLinkageTurnSubsystem(5, true, 18, false, -4, 4);
  private final ParallelLinkageTurnCommand turnCmnd = new ParallelLinkageTurnCommand(turnSubsystem, false);
  */
  private final ParallelLinkageTurnSubsystem turnSubManual = new ParallelLinkageTurnSubsystem(5);
  /*Current ratios with P = 0.018 D = 0.015
   * Desired: 0.2 Acheived: 0.76
   * Desired: 0.5 Achieved: 1   * Desired: 0.5(used 0.2) Achieved 0.5
   * Desired: 1 Acjieved: 1.357     * Desired: 1(used 0.5) Achieved 1
   * Desired: 1.5 ACchieved: 1.714   * Desired: 1.5(used 1) Achieved 1.5 using D = 0.01
   * Desired: 2 Achieved: -2.5    * Desired: 2(used 1.5) Achieved 1.976 using D = 0.01
   *                              *  Desired: 4 Achieved:  4.23   Using D = 0.01
   *                               * Desired: 5 achieved: 5. 143  using D = 0.01  Therefore about 5 rotations of Neo is aproximately 1/4 rotation of big boy
   */                               
  //private final SpringAssemblyShooterSubsystem shooter = new SpringAssemblyShooterSubsystem(1);
  
  /*
   * Testing other idea for turning rotationg for parallel Linkage
   */
  //private final SpringAssemblyShooterCommand shooterCommand = new SpringAssemblyShooterCommand(shooter);
  //private final ShooterLinkageMoverSubsystem linkage = new ShooterLinkageMoverSubsystem();
  //private final ShooterLinkageMoverCommand linkageCommand = new ShooterLinkageMoverCommand(linkage);  private final Joystick joystick1 = new Joystick(0);
  //private final ExtensionSubsystem extension = new ExtensionSubsystem();
  //private final ExtensionCommand extensionCommand = new ExtensionCommand(extension);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController armController = new CommandXboxController(1);
  //InstantCommand intakeOut = new InstantCommand(() -> intake.setIntakePower(-0.5));


  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    
    // extension.setDefaultCommand(extensionCommand);
    //shooter.setDefaultCommand(shooterCommand);
    //linkage.setDefaultCommand(linkageCommand);
   // driveTrain.setDefaultCommand(new DriveCommand(driveTrain, driverController/*driverController::getRightX, driverController::getLeftX, driverController::getLeftY*/));

    //Might not want to be creating a new instance of the command every time its called since its not "finishing any of the commands"
    
    
    /* 
    
    armController.y().onTrue(new ArmSetTargetCommand(arm, 2.57));
    armController.a().onTrue(new SequentialCommandGroup(new ArmSetTargetCommand(arm,0.23), new IntakeCommand(intake)));
    armController.b().onTrue(new ArmSetTargetCommand(arm, 1.7));
    armController.x().onTrue(new ArmSetTargetCommand(arm, 2.2));
    

    // armController.y().onTrue(new InstantCommand(() -> ext.setExtensionGoal(15)));
    // armController.x().onTrue(new InstantCommand(() -> ext.setExtensionGoal(0)));
    // armController.y().onTrue(new InstantCommand(() -> ext.setPower(0.2)));
    // armController.x().onTrue(new InstantCommand(() -> ext.setExtensionGoal(0)));


    //armController.x().onTrue(new InstantCommand(() -> arm.setRotationGoal(0.5)));

    
    /*Extension Code test 
    //driverController.y().onTrue(new ExtensionCommand(ext, 50, 0.1));
    //driverController.x().onTrue(new ExtensionCommand(ext, 0, 0.1));
    //driverController.a().onTrue(new ExtensionCommand(ext, 100, 0.1));
    //armController.x().onTrue(new InstantCommand(() -> arm.setRotationGoal(0.75)));

    
    
    //Need to turn off intake 
    driverController.rightTrigger().onTrue(new InstantCommand(() -> driveTrain.resetGyro()));
    // driveReveal.touchpad(test).onTrue(new InstantCommand(() -> driveReveal.setRumble(GenericHID.RumbleType.kBothRumble, 0.5)));
    //if (driveReveal.getTouchpadPressed()) driveReveal.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
    // armController.rightBumper().onTrue(new IntakeSetPowerCommand(intake, -1));
    // armController.leftBumper().onTrue(new IntakeSetPowerCommand(intake, -0.5));
    // armController.leftBumper().and(armController.rightBumper().onFalse(new IntakeSetPowerCommand(intake, 0)));
    armController.rightBumper().onTrue(new TimedIntakeCommand(intake, -0.8));
    armController.leftBumper().onTrue(new TimedIntakeCommand(intake, -0.3));
    armController.rightTrigger().onTrue(new ArmManualCommand(arm, true));
    armController.leftTrigger().onTrue(new ArmManualCommand(arm, false));
    armController.povDown().onTrue(new InstantCommand(() -> arm.resetArm()));
    
    //armController.leftTrigger().onTrue(intakeCommand);
    */
    

    configureBindings();
    //paraLinkage.setDefaultCommand(new ParallelLinkage(paraLinkage));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    //driverController.x().onTrue(new ParallelLinkageWristCommand(paraLinkage, true));
    //driverController.y().onTrue(new ParallelLinkageWristCommand(paraLinkage, false));
    //driverController.a().onTrue(new ParallelLinkageWristCommand(paraLinkage));
    /*

    driverController.leftTrigger().onTrue(new ParallelLinkageTurnCommand(turnSubsystem, true));
    driverController.rightTrigger().onTrue(new ParallelLinkageTurnCommand(turnSubsystem, false));

    */
    
    driverController.leftTrigger().onTrue(new ParallelLinkageTurnCommand(turnSubManual, () -> -driverController.getLeftTriggerAxis()));
    driverController.rightTrigger().onTrue(new ParallelLinkageTurnCommand(turnSubManual, () -> driverController.getRightTriggerAxis()));
    // for our old shooter we used .onTrue()

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
  public double getDriverRawAxis(int axis){
      return driverController.getRawAxis(axis);
  }

  
}

/*Testing for momentum of arm
 * 
 * L = mvr = Iw
 * cos(theta) * hypotenuse = length
 * Find ratio of w : theta, theta : length, theta to momentum
 * angualr accerleration = (2x - theta)/t^2
 * Stall torque = 2.6 Nm = m * r * Vt^2 
 * r = 26 in rn
 *
 * 
 * 
 * 
 * 
 */
