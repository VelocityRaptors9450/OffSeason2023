// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax backSpinMotor = new CANSparkMax(Constants.shooterBackSpinId, MotorType.kBrushless);
  //robot's perspectice left and right
  //private final CANSparkMax leftFrontSpinMotor = new CANSparkMax(Constants.shooterFrontSpinLId, MotorType.kBrushless);
  private final CANSparkMax frontSpinMotor = new CANSparkMax(Constants.shooterFrontSpinRId, MotorType.kBrushless);

  
  private final SparkMaxPIDController backSpin;
  private final SparkMaxPIDController frontSpin;
   

  private final AbsoluteEncoder backSpinEncoder = backSpinMotor.getAbsoluteEncoder(Type.kDutyCycle);
  private final AbsoluteEncoder frontSpinEncoder = frontSpinMotor.getAbsoluteEncoder(Type.kDutyCycle);
/*

  Pseuodcode

 */ 
  public ShooterSubsystem() {
    backSpinMotor.setInverted(true);
    //leftFrontSpinMotor.setInverted(true);
    //rightFrontSpinMotor.follow(leftFrontSpinMotor, true);
    frontSpinMotor.setInverted(true);
    
    backSpinMotor.setIdleMode(IdleMode.kCoast);
    //leftFrontSpinMotor.setIdleMode(IdleMode.kCoast);
    frontSpinMotor.setIdleMode(IdleMode.kCoast);
    backSpin = backSpinMotor.getPIDController();
    frontSpin = frontSpinMotor.getPIDController();
    backSpin.setFeedbackDevice(backSpinEncoder);
    frontSpin.setFeedbackDevice(frontSpinEncoder);

    backSpin.setP(0);
    backSpin.setD(0);
    frontSpin.setP(0);
    backSpin.setD(0);

    backSpin.setFF(0.0);
    

    //backSpinEncoder.setVelocityConversionFactor(2 * 0.0254 * Math.PI / 30); // rev / min   *   1 min / 60 sec   *  2Math.PI * r / rev    radius(2 inches) in meters
    //frontSpinEncoder.setVelocityConversionFactor(2 * 0.0254 * Math.PI / 30);
    //backSpinMotor.getEncoder().setVelocityConversionFactor(2 * 0.0254 * Math.PI / 30);
    //rightFrontSpinMotor.getEncoder().setVelocityConversionFactor(2 * 0.0254 * Math.PI / 30);

    
  }

  public void setBackSpinVoltage(double voltage){
    backSpinMotor.setVoltage(voltage);
  }

  public void setForwardSpinVoltage(double voltage){
    //leftFrontSpinMotor.setVoltage(voltage);
    frontSpinMotor.setVoltage(voltage);
  }
  public void setVoltage(double forSpinVolt, double backSpinVolt){
    setBackSpinVoltage(backSpinVolt);
    setForwardSpinVoltage(forSpinVolt);
  }

   

  public void setVelocity(double targetVel, double targetAcc){
    //leftFrontSpinMotor.set(frontSpinff.calculate(targetVel));
    frontSpinMotor.set(frontSpinff.calculate(targetVel, targetAcc));
    backSpinMotor.set(backSpinff.calculate(targetVel, targetAcc));
    
    //Hi My name is Krish
    //Hi Krish my name is NameNotFoundException
  }

  
  public double getVelocityFrontSpin(){
      return frontSpinEncoder.getVelocity();
    //return rightFrontSpinMotor.getEncoder().getVelocity();
  }

  public double getVelocityBackSpin(){
    return backSpinEncoder.getVelocity();
    //return backSpinMotor.getEncoder().getVelocity();
  }
  
  
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("FrontSpinPosition", getVelocityFrontSpin());
    SmartDashboard.putNumber("BackSpinPosition", getVelocityBackSpin()); 
  }
}
