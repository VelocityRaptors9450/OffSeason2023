// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax backSpinMotor = new CANSparkMax(Constants.shooterBackSpin, MotorType.kBrushless);
  //robot's perspectice left and right
  private final CANSparkMax leftFrontSpinMotor = new CANSparkMax(Constants.shooterFrontSpinL, MotorType.kBrushless);
  private final CANSparkMax rightFrontSpinMotor = new CANSparkMax(Constants.shooterFrontSpinR, MotorType.kBrushless);

  
  private final ProfiledPIDController backSpin = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
  private final ProfiledPIDController frontSpin = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0,0));
  public ShooterSubsystem() {
    backSpinMotor.setInverted(true);
    leftFrontSpinMotor.setInverted(true);
    rightFrontSpinMotor.follow(leftFrontSpinMotor, true);
    
    backSpinMotor.setIdleMode(IdleMode.kCoast);
    leftFrontSpinMotor.setIdleMode(IdleMode.kCoast);
    rightFrontSpinMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setBackSpinVoltage(double voltage){
    backSpinMotor.setVoltage(voltage);
  }

  public void setForwardSpinVoltage(double voltage){
    leftFrontSpinMotor.setVoltage(voltage);
    rightFrontSpinMotor.setVoltage(voltage);
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
