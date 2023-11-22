// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax backSpinMotor = new CANSparkMax(Constants.shooterBackSpin, MotorType.kBrushless);
  //robot's perspectice left and right
  private final CANSparkMax leftFrontSpinMotor = new CANSparkMax(Constants.shooterFrontSpinL, MotorType.kBrushless);
  private final CANSparkMax rightFrontSpinMotor = new CANSparkMax(Constants.shooterFrontSpinR, MotorType.kBrushless);

  
  private final PIDController backSpin = new PIDController(0, 0, 0);
  private final PIDController frontSpin = new PIDController(0, 0, 0);
   
  private final SimpleMotorFeedforward backSpinff = new SimpleMotorFeedforward(0.5, 0);// ka?
  private final SimpleMotorFeedforward frontSpinff = new SimpleMotorFeedforward(0, 0);// ka?

  private final RelativeEncoder backSpinEncoder = backSpinMotor.getAlternateEncoder(8192);
  private final RelativeEncoder frontSpinEncoder = rightFrontSpinMotor.getAlternateEncoder(8192);
/*

  Pseuodcode

 */ 
  public ShooterSubsystem() {
    backSpinMotor.setInverted(true);
    leftFrontSpinMotor.setInverted(true);
    rightFrontSpinMotor.follow(leftFrontSpinMotor, true);
    
    backSpinMotor.setIdleMode(IdleMode.kCoast);
    leftFrontSpinMotor.setIdleMode(IdleMode.kCoast);
    rightFrontSpinMotor.setIdleMode(IdleMode.kCoast);

    backSpinEncoder.setVelocityConversionFactor(4 * 0.0254 * Math.PI / 30); // rev / min   *   1 min / 60 sec   *  2Math.PI * r / rev    radius(4 inches) in meters
    frontSpinEncoder.setVelocityConversionFactor(4 * 0.0254 * Math.PI / 30);
    
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

  

  public void setVelocity(double targetVel, double targetAcc){
    leftFrontSpinMotor.set(frontSpinff.calculate(targetVel));
    backSpinMotor.set(backSpinff.calculate(targetVel));
    
    //Hi My name is Krish
    //Hi Krish my name is NameNotFoundException
  }

  
  public double getVelocityFrontSpin(){
    return frontSpinEncoder.getVelocity();
  }

  public double getVelocityBackSpin(){
    return backSpinEncoder.getVelocity();
  }
  
  
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
