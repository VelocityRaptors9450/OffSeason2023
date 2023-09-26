// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//TODO: Get length for PID use using getDistance
public class ExtensionSubsystem extends SubsystemBase {
  /** Creates a new ExtensionSubsystem. */
  private CANSparkMax extensionMotor = new CANSparkMax(Constants.extensionId,MotorType.kBrushless);
  PIDController pid;

  public ExtensionSubsystem() {
    extensionMotor.setIdleMode(IdleMode.kBrake);
  }
  //gear ratio 13.5 rotations for 1 full metal rotation
  public void ExtensionPID(double targetDistance, double targetProportion){
    pid = new PIDController(targetProportion, 0,0);
    setPower(pid.calculate(extensionMotor.getEncoder().getPosition(), targetDistance));
  }
  public void setPower(double power){
    extensionMotor.set(power);
  }
  public double currentPosition(){
    return extensionMotor.getEncoder().getPosition();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
