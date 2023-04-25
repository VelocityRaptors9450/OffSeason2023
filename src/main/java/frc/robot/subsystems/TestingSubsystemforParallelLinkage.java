// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestingSubsystemforParallelLinkage extends SubsystemBase {
  /** Creates a new TestingSubsystemforParallelLinkage. */
  private final CANSparkMax testMotorLinkage;
  public TestingSubsystemforParallelLinkage() {
    testMotorLinkage = new CANSparkMax(4, MotorType.kBrushless);
  }

  public void run(double power){
    testMotorLinkage.set(power);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
