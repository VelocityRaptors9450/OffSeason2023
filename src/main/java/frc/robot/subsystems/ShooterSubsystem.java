// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax backSpinMotor = new CANSparkMax(Constants.shooterBackSpin, MotorType.kBrushless);
  //robot's perspectice left and right
  private final CANSparkMax leftFrontSpinMotor = new CANSparkMax(Constants.shooterFrontSpinL, MotorType.kBrushless);
  private final CANSparkMax rightFrontSpinMotor = new CANSparkMax(Constants.shooterFrontSpinR, MotorType.kBrushless);
 
  public ShooterSubsystem() {
    leftFrontSpinMotor.follow(rightFrontSpinMotor, true);
    backSpinMotor.setInverted(true);
  }

  public void setBackSpinVoltage(double voltage){
    backSpinMotor.setVoltage(voltage);
  }

  public void setForwardSpinVoltage(double voltage){
    leftFrontSpinMotor.setVoltage(voltage);
  }
  public void setVoltage(double forSpinVolt, double backSpinVolt){
    setBackSpinVoltage(backSpinVolt);
    setForwardSpinVoltage(forSpinVolt);
  }
  

  
  
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
