// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpringAssemblyShooterSubsystem extends SubsystemBase {
  /** Creates a new SpringAssemblyShooter. */
  public CANSparkMax outputRight = new CANSparkMax(7, MotorType.kBrushless);
  public CANSparkMax outputLeft = new CANSparkMax(13, MotorType.kBrushless);
  
  private double power = 0.0;
  public SpringAssemblyShooterSubsystem(double power) {
    this.power = power;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shooting() {
    outputRight.set(power);
    outputLeft.set(-power);
  }
}
