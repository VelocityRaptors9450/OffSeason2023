// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  public static final double kMaxSpeed = Constants.Speeds.MaxSpeed; // 3 meters per second
  public static final double kMaxAngularSpeed = Constants.Speeds.MaxAngularSpeed; // 1/2 rotation per second (Math.PI)
  private boolean fieldRelative = true; // true for tele, false for auto


  private final Translation2d m_frontLeftLocation = new Translation2d(-Constants.baseWidth / 2, Constants.baseLength / 2);
  private final Translation2d m_frontRightLocation = new Translation2d(Constants.baseWidth / 2, Constants.baseLength / 2);
  private final Translation2d m_backLeftLocation = new Translation2d(-Constants.baseWidth / 2, -Constants.baseLength / 2);
  private final Translation2d m_backRightLocation = new Translation2d(Constants.baseWidth / 2, -Constants.baseLength / 2);

  private final SwerveModule m_frontLeft = new SwerveModule(Constants.flDriveId, Constants.flTurnId, false, false, Constants.flAbsoluteId, Constants.flAbsoluteEncoderOffset, false);
  // private final SwerveModule m_frontRight = fieldRelative ? new SwerveModule(Constants.frDriveId, Constants.frTurnId, true, false, Constants.frAbsoluteId, Constants.frAbsoluteEncoderOffset, false) : new SwerveModule(Constants.frDriveId, Constants.frTurnId, false, false, Constants.frAbsoluteId, Constants.frAbsoluteEncoderOffset, false);
  private final SwerveModule m_frontRight = new SwerveModule(Constants.frDriveId, Constants.frTurnId, true, false, Constants.frAbsoluteId, Constants.frAbsoluteEncoderOffset, false);

  
  private final SwerveModule m_backLeft = new SwerveModule(Constants.blDriveId, Constants.blTurnId, false, false, Constants.blAbsoluteId, Constants.blAbsoluteEncoderOffset, false);
  private final SwerveModule m_backRight = new SwerveModule(Constants.brDriveId, Constants.brTurnId, false, false, Constants.brAbsoluteId, Constants.brAbsoluteEncoderOffset, false);

  private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(Constants.gyroId, "rio");


 // can you access the other files
  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          pigeon.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public DriveTrain() {
    pigeon.reset();
  }
  public void resetGyro(){
    pigeon.reset();
  }
  public void fieldRelativeSwitch(){
    fieldRelative = !fieldRelative;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
    double xSpeed, double ySpeed, double rot, double periodSeconds) {
    SwerveModuleState[] swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fromDiscreteSpeeds(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, pigeon.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        pigeon.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  // rotate to target degrees at specified power
  public void rotateToHeading(double target, double power, double periodSeconds) {
    // controls where bang bang controller shuts off
    double buffer = 1;

    // get the difference between the current and target position,
    // wraping if necessary
    double distance = (pigeon.getYaw() % 360) - target; // error
    if (distance < -180) {
      distance += 360;
    } else if (distance > 180) {
      distance -= 360;
    }

    SmartDashboard.putNumber("Rotate Distance:", distance);

    // rotate towards the target until withing buffer
    if (Math.abs(distance) < buffer) {
      drive(0, 0, 0, periodSeconds);
    } else if (distance > 0 ) {
      drive(0, 0, power, periodSeconds);
    } else if (distance < 0) {
      drive(0, 0, -power, periodSeconds);
    }
  }
  
      
    
  // https://github.com/wpilibsuite/allwpilib/blob/a0c029a35b75e6327c2acb48002db021d9479ed7/wpimath/src/main/java/edu/wpi/first/math/kinematics/ChassisSpeeds.java#L61
  public static ChassisSpeeds fromDiscreteSpeeds(ChassisSpeeds discreteSpeeds, double dtSeconds) {
    return fromDiscreteSpeeds(
        discreteSpeeds.vxMetersPerSecond,
        discreteSpeeds.vyMetersPerSecond,
        discreteSpeeds.omegaRadiansPerSecond,
        dtSeconds);
  }

  public static ChassisSpeeds fromDiscreteSpeeds(
      double vxMetersPerSecond,
      double vyMetersPerSecond,
      double omegaRadiansPerSecond,
      double dtSeconds) {
    var desiredDeltaPose =
        new Pose2d(
            vxMetersPerSecond * dtSeconds,
            vyMetersPerSecond * dtSeconds,
            new Rotation2d(omegaRadiansPerSecond * dtSeconds));
    var twist = new Pose2d().log(desiredDeltaPose);
    return new ChassisSpeeds(twist.dx / dtSeconds, twist.dy / dtSeconds, twist.dtheta / dtSeconds);
  }

  public void setDrivePower(double power){
    m_frontLeft.setDrivePower(power);
    m_frontRight.setDrivePower(power);
    m_backLeft.setDrivePower(power);
    m_backRight.setDrivePower(power);

  }

  public void setTurnPower(double power){
    m_frontLeft.setTurningPower(power);
    m_frontRight.setTurningPower(power);
    m_backLeft.setTurningPower(power);
    m_backRight.setTurningPower(power);

  }
  @Override
  public void periodic() {
      SmartDashboard.putNumber("Front Right Position:", m_frontRight.getAbsRad());
      SmartDashboard.putNumber("Front Left Position:", m_frontLeft.getAbsRad());
      SmartDashboard.putNumber("Back Right Position:", m_backRight.getAbsRad());
      SmartDashboard.putNumber("Back Left Position:", m_backLeft.getAbsRad());
      SmartDashboard.putNumber("Pigeon", pigeon.getYaw());
      SmartDashboard.putNumber("Pigeon Cosine", pigeon.getRotation2d().getCos());
      SmartDashboard.putNumber("Pigeon Sine", pigeon.getRotation2d().getSin());

  }

}
//Hello World!