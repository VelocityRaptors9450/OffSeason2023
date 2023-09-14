// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestsSubsystem extends SubsystemBase {
  /** Creates a new TestsSubsystem. */
  private CANSparkMax speed = new CANSparkMax(1,MotorType.kBrushless);
  private CANSparkMax rotation = new CANSparkMax(2,MotorType.kBrushless);

  
  public void speedPower(double pow){
    speed.set(pow * 0.3);
  }
  public void rotPower(double pow){
    rotation.set(pow * 0.3);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
