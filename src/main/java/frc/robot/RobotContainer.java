// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.ArmManualCommand;
import frc.robot.commands.ArmSetTargetCommand;
import frc.robot.commands.ArmWristSetTargetCommand;
import frc.robot.commands.ExtensionCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeCommandWrist;
import frc.robot.commands.IntakeSetPowerCommand;
import frc.robot.commands.NewRotationCommand;
import frc.robot.commands.SetArmHeightPreset;
import frc.robot.commands.TimedIntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.RotationSubsystem;
import frc.robot.subsystems.ArmSubsystem.Height;

import java.time.Instant;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

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
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here..

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    
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

  
}
