// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.HashMap;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.MACAddress;

public class Robot extends TimedRobot {

  public double getVoltage(){
    return /*PDP.getVoltage()*/ 12;
  }


  /* Command Declarations */
  private Command frontLeft;
  private Command frontRight;
  private Command backLeft;
  private Command backRight;

  /* Autonomous Chooser Declarations */
  private Command m_autonomousCommand;
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  Timer time = new Timer();

   private static Robot instance = null;

   enum RobotType {
     COMPETITION,
     DRIVEBASE;
   } 
 
   public static Robot getInstance() {
     if (instance == null) instance = new Robot();
     return instance;
   }
 
   private final PowerDistribution PDP;
 
   public Controls controls;
   public Subsystems subsystems;
 
   private final RobotType robotType;
   public final Field2d field = new Field2d();
   
   protected Robot(RobotType type) {
     instance = this;
     PDP = new PowerDistribution(Hardware.PDP_ID, ModuleType.kRev);
     robotType = type;
   }
 
   protected Robot() {
     this(getTypeFromAddress());
   }
 
   public static final MACAddress COMPETITION_ADDRESS = MACAddress.of(0x33, 0x9d, 0xd1);
   public static final MACAddress PRACTICE_ADDRESS = MACAddress.of(0x28, 0x40, 0x82);
 
   private static RobotType getTypeFromAddress() {
     if (/*PRACTICE_ADDRESS.exists() */true) return RobotType.DRIVEBASE;
     else return RobotType.COMPETITION;
   }
 
   @Override
   public void robotInit() {
     LiveWindow.disableAllTelemetry();
     //LiveWindow.enableTelemetry(PDP);
 
     subsystems = new Subsystems();
     controls = new Controls(subsystems);
     
 
     if (subsystems.drivebaseSubsystem != null) {
       subsystems.drivebaseSubsystem.enableNoMotionCalibration();
     }
 
     Shuffleboard.startRecording();
 
     if (RobotBase.isReal()) {
       DataLogManager.start();
       DriverStation.startDataLog(DataLogManager.getLog(), true);
     }
 
     CommandScheduler.getInstance()
         .onCommandInitialize(
             command -> System.out.println("Command initialized: " + command.getName()));
     CommandScheduler.getInstance()
         .onCommandInterrupt(
             command -> System.out.println("Command interrupted: " + command.getName()));
     CommandScheduler.getInstance()
         .onCommandFinish(command -> System.out.println("Command finished: " + command.getName()));
 
     SmartDashboard.putData("Field", field);
     SmartDashboard.putData(CommandScheduler.getInstance());
     SmartDashboard.putData(subsystems.drivebaseSubsystem);
     DriverStation.silenceJoystickConnectionWarning(true);
 
     PathPlannerServer.startServer(5811);
 
     logRobotInfo();



    /* Initializing the Autonomous Chooser (stuff) */
    // Adds the options for the auto chooser.
    m_autoChooser.addOption("Front Left", frontLeft);
    m_autoChooser.addOption("Front Right", frontRight);
    m_autoChooser.addOption("Back Left", backLeft);
    m_autoChooser.addOption("Back Right", backRight);

    // Puts the auto chooser into it's own tab on Shuffleboard.
    ShuffleboardTab autoTab =
      Shuffleboard.getTab("Auto");

    Shuffleboard.getTab("Auto")
      .add("Autonomous Select:", m_autoChooser)
      .withWidget(BuiltInWidgets.kSplitButtonChooser)
      .withSize(3, 1);


  
   }
 
   private void logRobotInfo() {
     try {
       File gitInfoFile = new File(Filesystem.getDeployDirectory(), "git-info.txt");
       System.out.println("Git info:\n" + Files.readString(gitInfoFile.toPath()));
     } catch (IOException e) {
       DriverStation.reportWarning("Could not open git info file", true);
     }
   }
 
   public SwerveAutoBuilder getAutoBuilder(HashMap<String, Command> eventMap) {
     if (subsystems.drivebaseSubsystem != null) {
       return new SwerveAutoBuilder(
           subsystems.drivebaseSubsystem::getPose, // Pose2d supplier
           subsystems.drivebaseSubsystem
               ::resetPose, // Pose2d consumer, used to reset odometry at the beginning of
           // auto
           subsystems.drivebaseSubsystem.getKinematics(), // SwerveDriveKinematics
           new PIDConstants(
               5.0, 0.0,
               0.0), // PID constants to correct for translation error (used to create the X and
           // Y
           // PID controllers)
           new PIDConstants(
               3.0, 0.0,
               0.0), // PID constants to correct for rotation error (used to create the rotation
           // controller)
           subsystems.drivebaseSubsystem
               ::drive, // Module states consumer used to output to the drive subsystem
           eventMap,
           true, // Should the path be automatically mirrored depending on alliance color.
           // Optional, defaults to true
           subsystems
               .drivebaseSubsystem // The drive subsystem. Used to properly set the requirements
           // of
           // path following commands
           );
     } else {
       return null;
     }
   }
   

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  
  @Override
  public void autonomousInit() {
    time.restart();


    // frontLeft = new
    // frontRight = new
    // backLeft = new
    // backRight = new

    // m_autonomousCommand = m_autoChooser.getSelected();

    //schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
        m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    time.restart();

	Shuffleboard.startRecording();

	if (subsystems.drivebaseSubsystem != null) {
		subsystems.drivebaseSubsystem.setUseVisionMeasurements(true);
	}
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }    
  }

  @Override
	public void teleopExit() {
		CommandScheduler.getInstance().cancelAll();
		if (subsystems.drivebaseSubsystem != null) {
			subsystems.drivebaseSubsystem.stopAllMotors();
		}
		
  }
  
  int _loopCount = 0;
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}