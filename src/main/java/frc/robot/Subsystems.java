package frc.robot;

import static frc.robot.Subsystems.SubsystemConstants.*;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.DrivebaseSubsystem;


public class Subsystems {
	public static class SubsystemConstants {
		public static final boolean IS_COMP = true;
		public static final boolean DRIVEBASE_ENABLED = true;
		public static final boolean ARM_ENABLED = IS_COMP && true;
		public static final boolean INTAKE_ENABLED = IS_COMP && true;
		public static final boolean VISION_ENABLED = true;
		public static final boolean LED_ENABLED = IS_COMP && true;
		public static final boolean ARM_LED_ENABLED = IS_COMP && true;
		public static final boolean DRIVER_VIS_ENABLED = true;
	}

	public DrivebaseSubsystem drivebaseSubsystem;

	public SwerveDrivePoseEstimator poseEstimator;

	public Subsystems() {
		SwerveModulePosition[] pseudoPositions = new SwerveModulePosition[4];
		SwerveModulePosition defaultPosition = new SwerveModulePosition(0.0, new Rotation2d());
		for (int pseudoPosition = 0; pseudoPosition < pseudoPositions.length; pseudoPosition++) {
			pseudoPositions[pseudoPosition] = defaultPosition;
		}

		poseEstimator =
				new SwerveDrivePoseEstimator(
						DrivebaseSubsystem.kinematics, new Rotation2d(), pseudoPositions, new Pose2d());
		Field2d field = Robot.getInstance().field;

		if (DRIVEBASE_ENABLED) {
			drivebaseSubsystem = new DrivebaseSubsystem(poseEstimator, field);
		}
		
		
	}
}
