// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.ExtensionCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeStopAndResetCommand;
import frc.robot.commands.NewRotationCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.RotationSubsystem;

import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
  private IntakeSubsystem intake = new IntakeSubsystem();
  
  private ExtensionSubsystem ext = new ExtensionSubsystem();
  //private TestsSubsystem motorTest = new TestsSubsystem();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(1);
  private final CommandXboxController armController = new CommandXboxController(0);
  private IntakeCommand intakeCommand = new IntakeCommand(intake);


  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //driveTrain.setDefaultCommand(new DriveCommand(driveTrain, driverController::getRightX, driverController::getLeftX, driverController::getLeftY));

    //Might not want to be creating a new instance of the command every time its called since its not "finishing any of the commands"
    //armController.y().onTrue(new NewRotationCommand(arm, 0, 0));
    // armController.x().onTrue(new NewRotationCommand(arm, 0.75));
    // armController.a().onTrue(new NewRotationCommand(arm, 0));
    //armController.y().onTrue(new InstantCommand(() -> arm.setRotationGoal(1.5)));
    armController.y().onTrue(new InstantCommand(() -> arm.changeCoast()));
    armController.y().onFalse(new InstantCommand(() -> arm.changeBrake()));
    //armController.x().onTrue(new InstantCommand(() -> arm.setRotationGoal(0.75)));
    //armController.a().onTrue(new InstantCommand(() -> arm.setRotationGoal(0)));
    // driverController.y().onTrue(new NewRotationCommand(arm, 1.5));
    // driverController.x().onTrue(new NewRotationCommand(arm, 0.75));
    // driverController.a().onTrue(new NewRotationCommand(arm, 0));
    armController.y().onTrue(new InstantCommand(() -> arm.setArmWristGoal(-0.7)));
    //armController.x().onTrue(new InstantCommand(() -> arm.setRotationGoal(0.5)));

    
    /*Extension Code test */
    //driverController.y().onTrue(new ExtensionCommand(ext, 50, 0.1));
    //driverController.x().onTrue(new ExtensionCommand(ext, 0, 0.1));
    //driverController.a().onTrue(new ExtensionCommand(ext, 100, 0.1));
    armController.x().onTrue(new InstantCommand(() -> arm.setRotationGoal(0.75)));

    

    //Need to turn off intake 
    // armController.leftBumper().onTrue(new InstantCommand(() -> intake.setIntakePower(-0.5)));
    // armController.leftBumper().onFalse(new IntakeStopAndResetCommand(intake));
    // armController.leftTrigger().onTrue(intakeCommand);
    
    

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
    /*
    TrajectoryConfig config = new TrajectoryConfig(1, 1);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            Constants.kPThetaController, 0, 0, Constants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            driveTrain::getPose, // Functional interface to feed supplier
            Constants.kDriveKinematics,

            // Position controllers
            new PIDController(Constants.kPXController, 0, 0),
            new PIDController(Constants.kPYController, 0, 0),
            thetaController,
            driveTrain::setModuleStates,
            driveTrain);
      */




    // An example command will be run in autonomous
    return null;
  }
  public double getDriverRawAxis(int axis){
      return driverController.getRawAxis(axis);
  }

  
}
