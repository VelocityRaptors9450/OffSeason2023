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
import frc.robot.commands.ArmSetTargetCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class Subsystems {
	public static class SubsystemConstants {
		public static final boolean IS_COMP = true;
		public static final boolean DRIVEBASE_ENABLED = false;
		public static final boolean ARM_ENABLED = IS_COMP && true;
		public static final boolean INTAKE_ENABLED = IS_COMP && true;
		public static final boolean VISION_ENABLED = true;
		public static final boolean LED_ENABLED = IS_COMP && true;
		public static final boolean ARM_LED_ENABLED = IS_COMP && true;
		public static final boolean DRIVER_VIS_ENABLED = true;
	}



	public ArmSubsystem arm;

	public IntakeSubsystem intake;

	public Subsystems() {
		

		

		arm = new ArmSubsystem();
		intake = new IntakeSubsystem();
		
		
	}
}
