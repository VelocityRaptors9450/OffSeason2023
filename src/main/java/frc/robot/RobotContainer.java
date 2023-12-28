// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import frc.robot.commands.DriveCommand;

import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeSetPowerCommand;
import frc.robot.commands.LimelightTurretCommand;
//import frc.robot.commands.LimelightTurretCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TimedIntakeCommand;
import frc.robot.subsystems.RotationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.ManualDriveCommand;

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

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightTurretSubsystem;
//import frc.robot.subsystems.LimelightTurretSubsystem;
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
  // The robot's subsystems and commands are defined here...
  //public DriveTrain driveTrain = new DriveTrain();

  public ShooterSubsystem shooter = new ShooterSubsystem();
  public ShooterCommand shootComm;
  public LimelightTurretSubsystem turret = new LimelightTurretSubsystem();
  //private ExtensionSubsystem ext = new ExtensionSubsystem();
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final EventLoop test = new EventLoop();
  //private final CommandXboxController driverController = new CommandXboxController(0);
  
  private final CommandXboxController turretController = new CommandXboxController(1);
  //private IntakeCommand intakeCommand = new IntakeCommand(intake);
  //InstantCommand intakeOut = new InstantCommand(() -> intake.setIntakePower(-0.5));
  

  double wristScorePos = 0.46;
  double armScorePos = 0.35;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //driveTrain.setDefaultCommand(new DriveCommandSuppliers(driveTrain, driverController::getLeftY, 
                                                          //driverController::getLeftX, driverController::getRightX, 
                                                          //() -> driverController.x().getAsBoolean(), () -> driverController.rightBumper().getAsBoolean(), () -> driverController.getHID().getLeftStickButtonPressed(), () -> driverController.getHID().getRightStickButtonPressed()));

    //10, 12 max
    // shooter.setDefaultCommand(new ShooterCommand(shooter,20,0,3, 3));
    turretController.leftBumper().onTrue(new InstantCommand(() -> shooter.shootToPos(50)));
    turretController.leftBumper().onFalse(new InstantCommand(() -> shooter.setVelocity(0,0)));
    // conditional in the below parameter is to set the goal as the farthest from current pos (either .25 or .75 encoder)
    turretController.rightBumper().onTrue(new InstantCommand(() -> turret.setFlipTrue_Goal(Math.abs(turret.getTurretPosAngle() - 0.25 * 360.0) > Math.abs(turret.getTurretPosAngle() - 0.75 * 360.0) ? 0.25 * 360.0 : 0.75 * 360.0)));
    turret.setDefaultCommand(new LimelightTurretCommand(turret, turretController::getLeftX));
    
  
    
    //armController.leftBumper().onTrue(new InstantCommand(()-> shooter.setVoltage(2,1.8)));
    //armController.leftBumper().onFalse(new InstantCommand(() -> shooter.setVoltage(0, 0)));
    

    
    
    
    //Might not want to be creating a new instance of the command every time its called since its not "finishing any of the commands"
    
    
    // right trigger = intake pos
    // left trigger = outake
    // left bumper = scoring pos

    //armController.rightTrigger().onTrue(new SequentialCommandGroup(new ArmWristSetTargetCommand(arm,0.03, 0.72), new IntakeCommand(intake)));
    //armController.leftBumper().onTrue(new InstantCommand(() -> arm.goToHeight()));

    //armController.a().onTrue(new SetArmHeightPreset(arm, Height.LOW));
    //armController.x().onTrue(new SetArmHeightPreset(arm, Height.MID));
    //armController.y().onTrue(new SetArmHeightPreset(arm, Height.HIGH));
    //armController.b().onTrue(new SetArmHeightPreset(arm, Height.GROUND));

    //armController.y().onTrue(new ArmSetTargetCommand(arm, 0.35));
    //armController.a().onTrue(new SequentialCommandGroup(new ArmSetTargetCommand(arm,0.03)/*, new IntakeCommand(intake)*/));
    //armController.b().onTrue(new ArmSetTargetCommand(arm, 0.5));

    //armController.x().onTrue(new ArmSetTargetCommand(arm, 2));
    

    
    //armController.x().onTrue(new InstantCommand(() -> arm.setRotationGoal(0.5)));

    
    /*Extension Code test */
    //driverController.y().onTrue(new ExtensionCommand(ext, 50, 0.1));
    //driverController.x().onTrue(new ExtensionCommand(ext, 0, 0.1));
    //driverController.a().onTrue(new ExtensionCommand(ext, 100, 0.1));
    //armController.x().onTrue(new InstantCommand(() -> arm.setRotationGoal(0.75)));

    
    
    //Need to turn off intake 
    // driverController.rightTrigger().onTrue(new InstantCommand(() -> driveTrain.resetGyro()));
    
    // armController.rightBumper().onTrue(new TimedIntakeCommand(intake, -0.8));
    //armController.leftTrigger().onTrue(new TimedIntakeCommand(intake, -0.3));
    // armController.rightTrigger().onTrue(new ArmManualCommand(arm, true));
    // armController.leftTrigger().onTrue(new ArmManualCommand(arm, false));
    // armController.povDown().onTrue(new InstantCommand(() -> arm.resetArm()));
    
    
    

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
  // public double getDriverRawAxis(int axis){
      // return driverController.getRawAxis(axis);
  // }

  
}
