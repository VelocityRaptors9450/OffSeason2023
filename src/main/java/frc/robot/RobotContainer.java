// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.NewRotationCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.RotationSubsystem;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
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
  public DriveTrain driveTrain = new DriveTrain();
  private ArmSubsystem arm = new ArmSubsystem();
  //private TestsSubsystem motorTest = new TestsSubsystem();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController armController = new CommandXboxController(1);

  //private final PS4Controller driverController2 = new PS4Controller(0);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //driveTrain.setDefaultCommand(new DriveCommand(driveTrain, driverController::getRightX, driverController::getLeftX, driverController::getLeftY));
    driverController.y().onTrue(new NewRotationCommand(arm, 1.5));
    driverController.x().onTrue(new NewRotationCommand(arm, 0.75));
    driverController.a().onTrue(new NewRotationCommand(arm, 0));

    
    //rotation.setDefaultCommand(new RotationCommand(rotation, driverController::getHID));
    //motorTest.setDefaultCommand(new TestsCommand(motorTest, driverController));
    // Configure the trigger bindings
    //driverController.a().onTrue(new NewRotationCommand(arm, 0));
    //driverController.y().onTrue(new NewRotationCommand(arm, 0));

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   * 
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.  

    // new JoystickButton(joystick1, 0).whileTrue(new SwerveTurningOrientationCmd(swerve, false));
    // new JoystickButton(joystick1, 0).whileTrue(new SwerveTurningOrientationCmd(swerve, false));

    
    //driverController.x().onTrue(new LinkageSlowCommand(shooter, 0.9, 5));
    //driverController.y().onTrue(new LinkageSlowCommand(shooter, 0, 4));
    //driverController.a().onTrue(new LinkageSlowCommand(shooter, 0.3, 3));



    

    


   
    
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
