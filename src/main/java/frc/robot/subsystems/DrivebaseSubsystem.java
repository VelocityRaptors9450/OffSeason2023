package frc.robot.subsystems;

import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.Robot;
import frc.robot.sim.PhysicsSim;
import frc.robot.util.ModuleUtil;
import frc.robot.util.PFFController;
import frc.robot.util.gyroscope.Gyroscope;
import frc.robot.util.gyroscope.NavXGyro;
import frc.robot.util.gyroscope.Pigeon2Gyro;
import frc.robot.util.motorcontroller.BrushlessSparkMaxController;
import frc.robot.util.motorcontroller.MotorController;
import frc.robot.util.motorcontroller.MotorController.MotorControlMode;
import frc.robot.util.motorcontroller.MotorController.MotorNeutralMode;
import frc.robot.util.motorcontroller.TalonFXController;

public class DrivebaseSubsystem extends SubsystemBase {

	// ordered from front left, front right, back left, back right
	private static final Rotation2d[] COMP_DRIVEBASE_ENCODER_OFFSETS = {
		// Rotation2d.fromRadians(5.36431997269392),
		// Rotation2d.fromRadians(5.815309412777424),
		// Rotation2d.fromRadians(2.656849354505539),
		// Rotation2d.fromRadians(1.958889506757259)
		Rotation2d.fromRadians(4.84277734735497),
		Rotation2d.fromRadians(4.207709301170314),
		Rotation2d.fromRadians(1.3299),
		Rotation2d.fromRadians(3.77972866135022)
		
	};

	public static boolean IS_COMP = true; // just so i don't need to bother anymore with removing is_comp stuff
		// max drive speed is from SDS website and not calculated with robot weight
	public static final double MAX_DRIVE_SPEED_METERS_PER_SEC = 0.5; // was 4.4196
	// this is calculated as rotations_per_sec = velocity/(2*pi*turning radius (diagonal diameter))
	public static final Rotation2d MAX_ROTATIONS_PER_SEC =
			Rotation2d.fromRotations(0.2); // was 0.8574

	// magic number found by trial and error (aka informal characterization)
	private static final double ODOMETRY_ADJUSTMENT = 0.973748457;
	// 4 inches * odemetry adjustment
	private static final double WHEEL_DIAMETER_METERS =
			0.1016 * ODOMETRY_ADJUSTMENT;
	private static final double DRIVE_REDUCTION = (45.0 / (15.0 / (17.0 / (27.0 / (50.0 / 16.0)))));
	//if gear ratio is 14:50 then 27:17 then 15:45 where before ":" is driving gear and after it is driven gear
	private static final double DRIVE_REDUCTION_GEAR_RATIO = (45.0 / (15.0 / (17.0 / (27.0 / (50.0 / 14.0))))); 
	// steer reduction is the conversion from rotations of motor to rotations of the wheel
	// module rotation * STEER_REDUCTION = motor rotation
	public static final double STEER_REDUCTION =
			150.0 / 7.0;

	private static final double TIP_F = 0.02;
	private static final double TIP_P = 10;
	private static final double TIP_TOLERANCE = 2.5;

	private static final double DRIVE_VELOCITY_COEFFICIENT =
			DRIVE_REDUCTION / (Math.PI * WHEEL_DIAMETER_METERS); // meters to motor rotations

	// Balance controller is in degrees
	private final PFFController<Double> balanceController;

	private SwerveModuleState[] currentStates;

	private final MotorController[] moduleDriveMotors =
			new BrushlessSparkMaxController[] {
			new BrushlessSparkMaxController(Hardware.DRIVEBASE_FRONT_LEFT_DRIVE_MOTOR),
			new BrushlessSparkMaxController(Hardware.DRIVEBASE_FRONT_RIGHT_DRIVE_MOTOR),
			new BrushlessSparkMaxController(Hardware.DRIVEBASE_BACK_LEFT_DRIVE_MOTOR),
			new BrushlessSparkMaxController(Hardware.DRIVEBASE_BACK_RIGHT_DRIVE_MOTOR)
		};

	private final MotorController[] moduleAngleMotors =
			new BrushlessSparkMaxController[] {
			new BrushlessSparkMaxController(Hardware.DRIVEBASE_FRONT_LEFT_ANGLE_MOTOR),
			new BrushlessSparkMaxController(Hardware.DRIVEBASE_FRONT_RIGHT_ANGLE_MOTOR),
			new BrushlessSparkMaxController(Hardware.DRIVEBASE_BACK_LEFT_ANGLE_MOTOR),
			new BrushlessSparkMaxController(Hardware.DRIVEBASE_BACK_RIGHT_ANGLE_MOTOR)
		};

	private final WPI_CANCoder[] moduleEncoders = {
		new WPI_CANCoder(Hardware.DRIVEBASE_FRONT_LEFT_ENCODER_PORT),
		new WPI_CANCoder(Hardware.DRIVEBASE_FRONT_RIGHT_ENCODER_PORT),
		new WPI_CANCoder(Hardware.DRIVEBASE_BACK_LEFT_ENCODER_PORT),
		new WPI_CANCoder(Hardware.DRIVEBASE_BACK_RIGHT_ENCODER_PORT)
	};

	private final Rotation2d[] moduleOffsets =
			COMP_DRIVEBASE_ENCODER_OFFSETS;

	
	
	
	public static final double baseWidth = 0.4953;
	public static final double baseLength = 0.6477;

	// +X is along the direction of 0º and +Y is 90º CCW from +X. 
	// We want 0º to be straight forward, so +X is forward and +Y is 90º CCW from that, or left.
	private static final Translation2d[] moduleLocations =
			new Translation2d[] {
				new Translation2d(baseWidth / 2, baseLength / 2), // front left
				new Translation2d(baseWidth / 2, -baseLength / 2), // front right
				new Translation2d(-baseWidth / 2, baseLength / 2), // back left
				new Translation2d(-baseWidth / 2, -baseLength / 2) // back right
			};
	/* --> below is totes' version

			new Translation2d[] {
			new Translation2d(
					Units.inchesToMeters(12.375), Units.inchesToMeters(10.375)), // front left
			new Translation2d(
					Units.inchesToMeters(12.375), Units.inchesToMeters(-10.375)), // front right
			new Translation2d(
					Units.inchesToMeters(-12.375), Units.inchesToMeters(10.375)), // back left
			new Translation2d(
					Units.inchesToMeters(-12.375), Units.inchesToMeters(-10.375)) // back right
		};

	*/

	public static final SwerveDriveKinematics kinematics =
			new SwerveDriveKinematics(
					moduleLocations[0], moduleLocations[1], moduleLocations[2], moduleLocations[3]);

	private Gyroscope gyroscope;

	private final SwerveDrivePoseEstimator poseEstimator;
	private final SwerveDriveOdometry drivebaseOnlyOdometry;
	private Pose2d pose;

	private final Field2d field;
	private final FieldObject2d odometryOnlyFieldObject;
	private final FieldObject2d sharedPoseEstimatorFieldObject;

	private BooleanSubscriber useVisionMeasurementsSubscriber;
	private BooleanPublisher useVisionMeasurementsPublisher;

	private DoublePublisher frontLeftActualVelocityPublisher;
	private DoublePublisher frontRightActualVelocityPublisher;
	private DoublePublisher backLeftActualVelocityPublisher;
	private DoublePublisher backRightActualVelocityPublisher;

	private DoublePublisher frontLeftTargetVelocityPublisher;
	private DoublePublisher frontRightTargetVelocityPublisher;
	private DoublePublisher backLeftTargetVelocityPublisher;
	private DoublePublisher backRightTargetVelocityPublisher;

	private DoublePublisher frontLeftPercentPublisher;
	private DoublePublisher frontRightPercentPublisher;
	private DoublePublisher backLeftPercentPublisher;
	private DoublePublisher backRightPercentPublisher;

	private DoublePublisher frontLeftCurrentPublisher;
	private DoublePublisher frontRightCurrentPublisher;
	private DoublePublisher backLeftCurrentPublisher;
	private DoublePublisher backRightCurrentPublisher;

	private DoublePublisher frontLeftActualAnglePublisher;
	private DoublePublisher frontRightActualAnglePublisher;
	private DoublePublisher backLeftActualAnglePublisher;
	private DoublePublisher backRightActualAnglePublisher;

	private DoublePublisher frontLeftTargetAnglePublisher;
	private DoublePublisher frontRightTargetAnglePublisher;
	private DoublePublisher backLeftTargetAnglePublisher;
	private DoublePublisher backRightTargetAnglePublisher;


	// PID THESE CONSTANTS NOT USED RN
	public static final double turnKp = 4.0027;//4.0027 //12.0027
	public static final double turnKd = 0.10234;

	public static final double driveKp = 0.034037; //0.034037
	public static final double driveKd = 0.0;

	// TODO: check actual PID values
	private PIDController compTranslationalPID = new PIDController(0.0007, 0, 0); // kp was 0.0007
	private PIDController compRotationalPID = new PIDController(0.08, 0, 0.5); // was 0.1
	private final double DEFAULT_COMP_TRANSLATIONAL_F = 0.000175 * 0;
	// old way of getting F
	// 1 / moduleDriveMotors[0].getFreeSpeedRPS();

	private DoubleSubscriber compTranslationalF;

	private NetworkTableInstance networkTableInstance;
	private NetworkTable networkTableDrivebase;

	private boolean xWheelToggle = false;

	public DrivebaseSubsystem(SwerveDrivePoseEstimator initialPoseEstimator, Field2d field) {
		this.field = field;
		odometryOnlyFieldObject = field.getObject("OdometryPosition");
		sharedPoseEstimatorFieldObject = field.getObject("SharedPoseEstimator");
		// configure network tables
		configureNetworkTables();

		gyroscope = new Pigeon2Gyro(Hardware.GYRO_PORT);
		
		gyroscope.startLogging();

		poseEstimator = initialPoseEstimator;
		drivebaseOnlyOdometry =
				new SwerveDriveOdometry(kinematics, gyroscope.getRawYaw(), getModulePositions());

		resetPose(new Pose2d(), gyroscope.getRawYaw());

		balanceController =
				PFFController.ofDouble(TIP_F, TIP_P)
						.setTargetPosition(gyroscope.getRawRoll().getDegrees())
						.setTargetPositionTolerance(TIP_TOLERANCE);

		// configure encoders offsets
		for (int i = 0; i < moduleEncoders.length; i++) {
			moduleEncoders[i].configFactoryDefault();
			moduleEncoders[i].configSensorInitializationStrategy(
					SensorInitializationStrategy.BootToAbsolutePosition);
		}

		// configure drive motors
		for (int i = 0; i < moduleDriveMotors.length; i++) {
			MotorController driveMotor = moduleDriveMotors[i];
			driveMotor.setNeutralMode(MotorController.MotorNeutralMode.BRAKE);

			if (IS_COMP) {
				driveMotor.setControlMode(MotorControlMode.VELOCITY);
				driveMotor.setPIDF(
						compTranslationalPID.getP(),
						compTranslationalPID.getI(),
						compTranslationalPID.getD(),
						compTranslationalF.get());
				driveMotor.setMeasurementPeriod(8);
			} else {
				driveMotor.setControlMode(MotorControlMode.VELOCITY);
				driveMotor.setPIDF(0.1, 0.001, 1023.0 / 20660.0, 0);
			}
			driveMotor.configCurrentLimit(30);
			driveMotor.flashMotor();
		}

		// configure angle motors
		for (int i = 0; i < moduleAngleMotors.length; i++) {
			MotorController steeringMotor = moduleAngleMotors[i];
			steeringMotor.configFactoryDefault();
			steeringMotor.setNeutralMode(MotorNeutralMode.BRAKE);
			// Configure PID values
			if (IS_COMP) {
				steeringMotor.setPIDF(
						compRotationalPID.getP(), compRotationalPID.getI(), compRotationalPID.getD(), 0);
			} else {
				steeringMotor.setPIDF(0.15, 0.00, 1.0, 0);
			}

			if (IS_COMP) {
				steeringMotor.setInverted(true);
			}

			steeringMotor.useIntegratedEncoder();
			steeringMotor.setIntegratedEncoderPosition(
					getModuleAngles()[i].getRotations() * STEER_REDUCTION);
			steeringMotor.configureOptimization();

			steeringMotor.setControlMode(MotorControlMode.POSITION);

			steeringMotor.flashMotor();
		}
	}

	/** Drives the robot using forward, strafe, and rotation. Units in meters */
	public void drive(
			double forward,
			double strafe,
			Rotation2d rotation,
			boolean fieldOriented,
			boolean autoBalance) {
		// Auto balancing will only be used in autonomous
		if (autoBalance) {
			forward -= balanceController.update(gyroscope.getRawRoll().getDegrees());
		}

		ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);

		if (fieldOriented) {
			chassisSpeeds =
					ChassisSpeeds.fromFieldRelativeSpeeds(
							forward, -strafe, rotation.getRadians(), gyroscope.getAngle());
		} else {
			chassisSpeeds = new ChassisSpeeds(forward, -strafe, rotation.getRadians());
		}

		drive(chassisSpeeds);
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		SwerveModuleState[] moduleStates = getModuleStates(chassisSpeeds);
		if (xWheelToggle) {
			if (Math.abs(chassisSpeeds.vxMetersPerSecond) <= 0.01
					&& Math.abs(chassisSpeeds.vyMetersPerSecond) <= 0.01
					&& Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= 0.01) {
				moduleStates[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
				moduleStates[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
				moduleStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
				moduleStates[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
			}
		}
		drive(moduleStates);
	}

	/** Drives the robot using states */
	public void drive(SwerveModuleState[] states) {
		SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_DRIVE_SPEED_METERS_PER_SEC);

		for (int i = 0; i < states.length; i++) {
			if (IS_COMP) {
				states[i] = SwerveModuleState.optimize(states[i], getModuleAngles()[i]);
			} else {
				// this optimize assumes that PID loop is not continuous
				states[i] = ModuleUtil.optimize(states[i], getModuleAngles()[i]);
			}
		}

		// Set motor speeds and angles
		for (int i = 0; i < moduleDriveMotors.length; i++) {
			moduleDriveMotors[i].set(
					states[i].speedMetersPerSecond * DRIVE_VELOCITY_COEFFICIENT); // set velocity
			SmartDashboard.putNumber("drive motor set", states[0].speedMetersPerSecond * DRIVE_VELOCITY_COEFFICIENT);
		}
		for (int i = 0; i < moduleAngleMotors.length; i++) {
			moduleAngleMotors[i].set(states[i].angle.getRotations() * STEER_REDUCTION);
			SmartDashboard.putNumber("angle motor set", states[i].angle.getRotations() * STEER_REDUCTION);

		}

		currentStates = states;

		frontLeftActualVelocityPublisher.set(
				moduleDriveMotors[0].getVelocity() / DRIVE_VELOCITY_COEFFICIENT);
		frontRightActualVelocityPublisher.set(
				moduleDriveMotors[1].getVelocity() / DRIVE_VELOCITY_COEFFICIENT);
		backLeftActualVelocityPublisher.set(
				moduleDriveMotors[2].getVelocity() / DRIVE_VELOCITY_COEFFICIENT);
		backRightActualVelocityPublisher.set(
				moduleDriveMotors[3].getVelocity() / DRIVE_VELOCITY_COEFFICIENT);

		frontLeftTargetVelocityPublisher.set(states[0].speedMetersPerSecond);
		frontRightTargetVelocityPublisher.set(states[1].speedMetersPerSecond);
		backLeftTargetVelocityPublisher.set(states[2].speedMetersPerSecond);
		backRightTargetVelocityPublisher.set(states[3].speedMetersPerSecond);

		frontLeftPercentPublisher.set(moduleDriveMotors[0].getPercentOutput());
		frontRightPercentPublisher.set(moduleDriveMotors[1].getPercentOutput());
		backLeftPercentPublisher.set(moduleDriveMotors[2].getPercentOutput());
		backRightPercentPublisher.set(moduleDriveMotors[3].getPercentOutput());

		frontLeftCurrentPublisher.set(moduleDriveMotors[0].getCurrentOutput());
		frontRightCurrentPublisher.set(moduleDriveMotors[1].getCurrentOutput());
		backLeftCurrentPublisher.set(moduleDriveMotors[2].getCurrentOutput());
		backRightCurrentPublisher.set(moduleDriveMotors[3].getCurrentOutput());

		frontLeftTargetAnglePublisher.set(states[0].angle.getDegrees());
		frontRightTargetAnglePublisher.set(states[1].angle.getDegrees());
		backLeftTargetAnglePublisher.set(states[2].angle.getDegrees());
		backRightTargetAnglePublisher.set(states[3].angle.getDegrees());

		if (Robot.isSimulation()) {
			ChassisSpeeds speeds = kinematics.toChassisSpeeds(states);
			// Sim runs 50 times per second
			updateSimAngle(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond / 50));
		}
	}

	/**
	 * Array with modules with front left at [0], front right at [1], back left at [2], back right at
	 * [3]
	 */
	public SwerveModuleState[] getModuleStates(ChassisSpeeds speeds) {
		return kinematics.toSwerveModuleStates(speeds);
	}

	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];

		for (int i = 0; i < moduleDriveMotors.length; i++) {
			positions[i] =
					new SwerveModulePosition(
							moduleDriveMotors[i].getIntegratedEncoderPosition() / DRIVE_VELOCITY_COEFFICIENT,
							Rotation2d.fromRotations(
									moduleAngleMotors[i].getIntegratedEncoderPosition() / STEER_REDUCTION));
		}

		return positions;
	}

	/** Returns the module angles using encoders */
	public Rotation2d[] getModuleAngles() {
		Rotation2d[] rotations = new Rotation2d[4];
		for (int i = 0; i < moduleAngleMotors.length; i++) {
			rotations[i] =
					Rotation2d.fromDegrees(
							(moduleEncoders[i].getAbsolutePosition() - moduleOffsets[i].getDegrees()));
		}
		return rotations;
	}

	/** Returns the absolute module angles using encoders (no offsets included) */
	public Rotation2d[] getAbsModuleAngles() {
		Rotation2d[] rotations = new Rotation2d[4];
		for (int i = 0; i < moduleAngleMotors.length; i++) {
			rotations[i] =
					Rotation2d.fromDegrees(
							(moduleEncoders[i].getAbsolutePosition()));
		}
		return rotations;
	}

	/** Returns the kinematics */
	public SwerveDriveKinematics getKinematics() {
		return kinematics;
	}

	public SwerveModuleState[] getCurrentStates() {
		return currentStates;
	}

	// public void runPower() {
	// 	for (BrushlessSparkMaxController moduleDriveMotor : moduleDriveMotors) {
	// 		moduleDriveMotor.setPower(0.1);
	// 	}
	// }

	public double getVelocity() {
		if (currentStates == null) {
			return 0.0;
		}
		return Math.sqrt(
				Math.pow(kinematics.toChassisSpeeds(getCurrentStates()).vxMetersPerSecond, 2)
						+ Math.pow(kinematics.toChassisSpeeds(getCurrentStates()).vyMetersPerSecond, 2));
	}

	/**
	 * Resets the gyroscope's angle to 0 After this is called, the radio (on bonk) or the intake (on
	 * comp) will be the robot's new global forward
	 */
	public void resetGyroAngle() {
		resetGyroAngle(gyroscope.getRawYaw());
	}

	public void resetGyroAngleWithOrientation(Rotation2d angle) {
		resetGyroAngle(gyroscope.getRawYaw().plus(angle));
	}

	/**
	 * Resets the robot's forward to the new angle relative to the radio (on bonk)
	 *
	 * @param angle The new forward
	 */
	public void resetGyroAngle(Rotation2d angle) {
		gyroscope.setAngleAdjustment(angle.unaryMinus());
	}

	public void disableNoMotionCalibration() {
		gyroscope.disableNoMotionCalibration();
	}

	public void enableNoMotionCalibration() {
		gyroscope.enableNoMotionCalibration();
	}

	/** Returns the robot's pose */
	public Pose2d getPose() {
		return pose;
	}

	/**
	 * Set's the robot's pose to the provided pose
	 *
	 * @param pose the new pose
	 */
	public void resetPose(Pose2d pose) {
		resetPose(pose, pose.getRotation());
	}

	private void resetPose(Pose2d pose, Rotation2d gyroAngle) {
		SwerveModulePosition[] modulePositions = getModulePositions();
		synchronized (poseEstimator) {
			poseEstimator.resetPosition(gyroAngle, modulePositions, pose);
		}
		drivebaseOnlyOdometry.resetPosition(gyroAngle, modulePositions, pose);
		this.pose = pose;
	}

	/**
	 * Reset's the robot's pose to (0, 0) with rotation of 0. <br>
	 * Also resets the gyroscope
	 */
	public void resetPose() {
		resetGyroAngle();
		resetPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
	}

	public void resetPoseToOdometryPose() {
		resetPose(drivebaseOnlyOdometry.getPoseMeters());
	}

	public void resetPoseToPoseEstimatorPose() {
		synchronized (poseEstimator) {
			resetPose(poseEstimator.getEstimatedPosition());
		}
	}

	public void toggleXWheels() {
		xWheelToggle = !xWheelToggle;
	}

	public void stopAllMotors() {
		for (MotorController motor : moduleDriveMotors) {
			motor.stop();
		}
		for (MotorController motor : moduleAngleMotors) {
			motor.stop();
		}
	}

	public void simInit(PhysicsSim sim) {
		for (int i = 0; i < moduleDriveMotors.length; i++) {
			moduleDriveMotors[i].simulationConfig(sim);
			moduleAngleMotors[i].simulationConfig(sim);
		}
	}

	/** Update pose to reflect rotation */
	private void updateSimAngle(Rotation2d rotation) {
		gyroscope.updateSimulatedAngle(rotation);
	}

	private void configureNetworkTables() {
		networkTableInstance = NetworkTableInstance.getDefault();
		networkTableDrivebase = networkTableInstance.getTable("Drivebase");

		BooleanTopic useVisionMeasurementsTopic =
				networkTableDrivebase.getBooleanTopic("Use vision measurements");
		useVisionMeasurementsTopic.setPersistent(true);
		useVisionMeasurementsSubscriber = useVisionMeasurementsTopic.subscribe(false);
		useVisionMeasurementsPublisher = useVisionMeasurementsTopic.publish();

		frontLeftActualVelocityPublisher =
				networkTableDrivebase.getDoubleTopic("Front left actual velocity").publish();
		frontRightActualVelocityPublisher =
				networkTableDrivebase.getDoubleTopic("Front right actual velocity").publish();
		backLeftActualVelocityPublisher =
				networkTableDrivebase.getDoubleTopic("Back left actual velocity").publish();
		backRightActualVelocityPublisher =
				networkTableDrivebase.getDoubleTopic("Back right actual velocity").publish();

		frontLeftTargetVelocityPublisher =
				networkTableDrivebase.getDoubleTopic("Front left target velocity").publish();
		frontRightTargetVelocityPublisher =
				networkTableDrivebase.getDoubleTopic("Front right target velocity").publish();
		backLeftTargetVelocityPublisher =
				networkTableDrivebase.getDoubleTopic("Back left target velocity").publish();
		backRightTargetVelocityPublisher =
				networkTableDrivebase.getDoubleTopic("Back right target velocity").publish();

		frontLeftPercentPublisher =
				networkTableDrivebase.getDoubleTopic("Front left target percent").publish();
		frontRightPercentPublisher =
				networkTableDrivebase.getDoubleTopic("Front right target percent").publish();
		backLeftPercentPublisher =
				networkTableDrivebase.getDoubleTopic("Back left target percent").publish();
		backRightPercentPublisher =
				networkTableDrivebase.getDoubleTopic("Back right target percent").publish();

		frontLeftCurrentPublisher =
				networkTableDrivebase.getDoubleTopic("Front left target current").publish();
		frontRightCurrentPublisher =
				networkTableDrivebase.getDoubleTopic("Front right target current").publish();
		backLeftCurrentPublisher =
				networkTableDrivebase.getDoubleTopic("Back left target current").publish();
		backRightCurrentPublisher =
				networkTableDrivebase.getDoubleTopic("Back right target current").publish();

		frontLeftActualAnglePublisher =
				networkTableDrivebase.getDoubleTopic("Front left actual angle").publish();
		frontRightActualAnglePublisher =
				networkTableDrivebase.getDoubleTopic("Front right actual angle").publish();
		backLeftActualAnglePublisher =
				networkTableDrivebase.getDoubleTopic("Back left actual angle").publish();
		backRightActualAnglePublisher =
				networkTableDrivebase.getDoubleTopic("Back right actual angle").publish();

		frontLeftTargetAnglePublisher =
				networkTableDrivebase.getDoubleTopic("Front left target angle").publish();
		frontRightTargetAnglePublisher =
				networkTableDrivebase.getDoubleTopic("Front right target angle").publish();
		backLeftTargetAnglePublisher =
				networkTableDrivebase.getDoubleTopic("Back left target angle").publish();
		backRightTargetAnglePublisher =
				networkTableDrivebase.getDoubleTopic("Back right target angle").publish();
		compTranslationalF =
				networkTableDrivebase
						.getDoubleTopic("Translational FF")
						.subscribe(DEFAULT_COMP_TRANSLATIONAL_F);

		// Set value once to make it show up in UIs
		useVisionMeasurementsPublisher.set(false);
		compTranslationalF.getTopic().publish().set(DEFAULT_COMP_TRANSLATIONAL_F);

		frontLeftActualVelocityPublisher.set(0.0);
		frontRightActualVelocityPublisher.set(0.0);
		backLeftActualVelocityPublisher.set(0.0);
		backRightActualVelocityPublisher.set(0.0);

		frontLeftTargetVelocityPublisher.set(0.0);
		frontRightTargetVelocityPublisher.set(0.0);
		backLeftTargetVelocityPublisher.set(0.0);
		backRightTargetVelocityPublisher.set(0.0);

		frontLeftPercentPublisher.set(0.0);
		frontRightPercentPublisher.set(0.0);
		backLeftPercentPublisher.set(0.0);
		backRightPercentPublisher.set(0.0);

		frontLeftCurrentPublisher.set(0.0);
		frontRightCurrentPublisher.set(0.0);
		backLeftCurrentPublisher.set(0.0);
		backRightCurrentPublisher.set(0.0);

		frontLeftActualAnglePublisher.set(0.0);
		frontRightActualAnglePublisher.set(0.0);
		backLeftActualAnglePublisher.set(0.0);
		backRightActualAnglePublisher.set(0.0);

		frontLeftTargetAnglePublisher.set(0.0);
		frontRightTargetAnglePublisher.set(0.0);
		backLeftTargetAnglePublisher.set(0.0);
		backRightTargetAnglePublisher.set(0.0);

		SmartDashboard.putData("Translational PID", compTranslationalPID);
		SmartDashboard.putData("Rotational PID", compRotationalPID);
	}

	private double oldTranslationalSetpoint = 0.0;
	private double oldRotationalSetpoint = 0.0;

	@Override
	public void periodic() {
		Rotation2d gyroAngle = gyroscope.getAngle();
		SwerveModulePosition[] modulePositions = getModulePositions();
		Pose2d combinedPose, odometryPose;
		synchronized (poseEstimator) {
			combinedPose = poseEstimator.update(gyroAngle, modulePositions);
		}
		odometryPose = drivebaseOnlyOdometry.update(gyroAngle, modulePositions);
		pose = useVisionMeasurementsSubscriber.get() ? combinedPose : odometryPose;
		sharedPoseEstimatorFieldObject.setPose(combinedPose);
		odometryOnlyFieldObject.setPose(odometryPose);
		field.setRobotPose(pose);

		if (compTranslationalPID.getSetpoint() != oldTranslationalSetpoint) {
			for (MotorController motor : moduleDriveMotors) {
				motor.setPIDF(
						compTranslationalPID.getP(),
						compTranslationalPID.getI(),
						compTranslationalPID.getD(),
						compTranslationalF.get());
				oldTranslationalSetpoint = compTranslationalPID.getSetpoint();
			}
		}
		if (compRotationalPID.getSetpoint() != oldRotationalSetpoint) {
			for (MotorController motor : moduleAngleMotors) {
				motor.setPIDF(
						compRotationalPID.getP(), compRotationalPID.getI(), compRotationalPID.getD(), 0);
				oldRotationalSetpoint = compRotationalPID.getSetpoint();
			}
		}

		Rotation2d[] moduleAngles = getModuleAngles();
		frontLeftActualAnglePublisher.set(moduleAngles[0].getDegrees());
		frontRightActualAnglePublisher.set(moduleAngles[1].getDegrees());
		backLeftActualAnglePublisher.set(moduleAngles[2].getDegrees());
		backRightActualAnglePublisher.set(moduleAngles[3].getDegrees());
		
		Rotation2d[] absModuleAngles = getAbsModuleAngles();
		SmartDashboard.putNumber("FLPos", absModuleAngles[0].getRadians());
		SmartDashboard.putNumber("FRPos", absModuleAngles[1].getRadians());
		SmartDashboard.putNumber("BLPos", absModuleAngles[2].getRadians());
		SmartDashboard.putNumber("BRPos", absModuleAngles[3].getRadians());
		
		
	}

	public void setUseVisionMeasurements(boolean useVisionMeasurements) {
		useVisionMeasurementsPublisher.set(useVisionMeasurements);
	}
}
