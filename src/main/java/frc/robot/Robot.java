package frc.robot;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.sim.PhysicsSim;
import frc.robot.util.MACAddress;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.HashMap;
import java.util.Optional;

public class Robot extends TimedRobot {
	/** Singleton Stuff */
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

	public double getVoltage() {
		return PDP.getVoltage();
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
	public void testInit() {}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		
	}

	@Override
	public void teleopInit() {
		Shuffleboard.startRecording();

		if (subsystems.drivebaseSubsystem != null) {
			subsystems.drivebaseSubsystem.setUseVisionMeasurements(true);
		}
	}

	@Override
	public void autonomousExit() {
		CommandScheduler.getInstance().cancelAll();
		subsystems.drivebaseSubsystem.stopAllMotors();
	}

	@Override
	public void teleopExit() {
		CommandScheduler.getInstance().cancelAll();
		if (subsystems.drivebaseSubsystem != null) {
			subsystems.drivebaseSubsystem.stopAllMotors();
		}
		
	}

	@Override
	public void simulationInit() {
		PhysicsSim sim = PhysicsSim.getInstance();
		if (Subsystems.SubsystemConstants.DRIVEBASE_ENABLED) {
			subsystems.drivebaseSubsystem.simInit(sim);
		}
		
	}

	@Override
	public void simulationPeriodic() {
		PhysicsSim.getInstance().run();
	}

	public RobotType getRobotType() {
		return robotType;
	}

	public boolean isCompetition() {
		return getRobotType() == RobotType.COMPETITION;
	}



	@Override
	public void disabledInit() {
		Shuffleboard.stopRecording();
		
	}

	@Override
	public void disabledPeriodic() {
		
	}

	@Override
	public void disabledExit() {

	}
}
