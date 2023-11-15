// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MathConstants;
import frc.robot.Constants.OperatorConstants;

public class ShooterSubsystem extends SubsystemBase {

  /* Motor Declarations */
  private final CANSparkMax backSpin = new CANSparkMax(OperatorConstants.ShooterBackSpin, MotorType.kBrushless);

  private final CANSparkMax frontSpin1 = new CANSparkMax(OperatorConstants.ShooterBackSpin, MotorType.kBrushless);
  private final CANSparkMax frontSpin2 = new CANSparkMax(OperatorConstants.ShooterBackSpin, MotorType.kBrushless);


  /* PID Declarations */
  private final ProfiledPIDController backSpinPID = new ProfiledPIDController(MathConstants.back_kp, MathConstants.back_ki, MathConstants.back_kd, null);


  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
