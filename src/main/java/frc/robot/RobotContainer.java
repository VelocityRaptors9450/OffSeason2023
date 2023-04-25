// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExtensionCommand;
import frc.robot.commands.ParallelLinkage;
// import frc.robot.commands.ShooterLinkageMoverCommand;
import frc.robot.commands.SpringAssemblyShooterCommand;
import frc.robot.subsystems.ExtensionSubsystem;
// import frc.robot.subsystems.ShooterLinkageMoverSubsystem;
import frc.robot.subsystems.SpringAssemblyShooterSubsystem;
import frc.robot.subsystems.TestingSubsystemforParallelLinkage;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  private final TestingSubsystemforParallelLinkage paraLinkage = new TestingSubsystemforParallelLinkage();
  private final SpringAssemblyShooterSubsystem shooter = new SpringAssemblyShooterSubsystem(1);
  //private final SpringAssemblyShooterCommand shooterCommand = new SpringAssemblyShooterCommand(shooter);
  //private final ShooterLinkageMoverSubsystem linkage = new ShooterLinkageMoverSubsystem();
  //private final ShooterLinkageMoverCommand linkageCommand = new ShooterLinkageMoverCommand(linkage);  private final Joystick joystick1 = new Joystick(0);
  //private final ExtensionSubsystem extension = new ExtensionSubsystem();
  //private final ExtensionCommand extensionCommand = new ExtensionCommand(extension);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    
    // extension.setDefaultCommand(extensionCommand);
    //shooter.setDefaultCommand(shooterCommand);
    //linkage.setDefaultCommand(linkageCommand);
    //configureBindings();
    paraLinkage.setDefaultCommand(new ParallelLinkage(paraLinkage));
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
   // driverController.a().onTrue(linkageCommand);
    //driverController.y().getAsBoolean();
    
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
