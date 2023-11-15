// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase
{
  public static final CANSparkMax fl = new CANSparkMax(Constants.flDriveId, MotorType.kBrushless);
  public static final CANSparkMax fr = new CANSparkMax(Constants.frDriveId, MotorType.kBrushless);
  public static final CANSparkMax bl = new CANSparkMax(Constants.blDriveId, MotorType.kBrushless);
  public static final CANSparkMax br = new CANSparkMax(Constants.brDriveId, MotorType.kBrushless);




  /** Creates a new DriveTrain. */
  public DriveTrain() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
