// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public final SwerveModule fl = new SwerveModule(
    DriveConstants.FL_MOVE_MOTOR_ID, 
    DriveConstants.FL_TURN_MOTOR_ID,
    false, 
    false, 
    DriveConstants.FL_ABSOLUTE_ENCODER, 
    0, 
    false);
    
  public final SwerveModule fr = new SwerveModule(
    DriveConstants.FR_MOVE_MOTOR_ID, 
    DriveConstants.FR_TURN_MOTOR_ID,
    false, 
    false, 
    DriveConstants.FR_ABSOLUTE_ENCODER, 
    0, 
    false);

  public final SwerveModule bl = new SwerveModule(
    DriveConstants.BL_MOVE_MOTOR_ID, 
    DriveConstants.BL_TURN_MOTOR_ID,
    false, 
    false, 
    DriveConstants.BL_ABSOLUTE_ENCODER, 
    0, 
    false);
  public final SwerveModule br = new SwerveModule(
    DriveConstants.BR_MOVE_MOTOR_ID, 
    DriveConstants.BR_TURN_MOTOR_ID,
    false, 
    false, 
    DriveConstants.BR_ABSOLUTE_ENCODER, 
    0, 
    false);
}
