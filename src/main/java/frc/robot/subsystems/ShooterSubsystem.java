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
  private final CANSparkMax leftFrontSpinMotor = new CANSparkMax(Constants.shooterFrontSpinLId, MotorType.kBrushless);
  private final CANSparkMax rightFrontSpinMotor = new CANSparkMax(Constants.shooterFrontSpinRId, MotorType.kBrushless);

  private final SimpleMotorFeedforward backSpinFF = new SimpleMotorFeedforward(0.0098,0.01025);
  private final SimpleMotorFeedforward frontSpinFF = new SimpleMotorFeedforward(0.00898, 0.01387);
  
  //private final SparkMaxPIDController backSpin, frontSpin;
    private final PIDController backSpin = new PIDController(0.0001,0,0);
    private final PIDController frontSpin = new PIDController(0.00021,0,0);

    

  private final AbsoluteEncoder backSpinEncoder = backSpinMotor.getAbsoluteEncoder(Type.kDutyCycle);
  private final AbsoluteEncoder frontSpinEncoder = rightFrontSpinMotor.getAbsoluteEncoder(Type.kDutyCycle);

  private ShooterPos shootPos = ShooterPos.CLOSE;
  //rpm values taken from interpolation/ from table we made.
  private double closeShootRpm = 0, mediumShootRpm = 0, farShootRpm = 0;
  
/*

  Pseuodcode

 */ 
  public ShooterSubsystem() {
    backSpinMotor.setInverted(true);
    leftFrontSpinMotor.setInverted(true);
    rightFrontSpinMotor.follow(leftFrontSpinMotor, true);
    
    backSpinMotor.setIdleMode(IdleMode.kBrake);
    leftFrontSpinMotor.setIdleMode(IdleMode.kBrake);
    rightFrontSpinMotor.setIdleMode(IdleMode.kBrake);
    
    //backSpin = backSpinMotor.getPIDController();
   // frontSpin = rightFrontSpinMotor.getPIDController();
    //backSpin.setFeedbackDevice(backSpinEncoder);
    //frontSpin.setFeedbackDevice(frontSpinEncoder);

   // backSpin.setP(0.001); 
   // backSpin.setD(0);
    //frontSpin.setP(0);
    //backSpin.setD(0);
    //ff is lowest power needed to shoot to closest distance
    //backSpin.setFF(0);
    //frontSpin.setFF(0);
    
    

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
    leftFrontSpinMotor.setVoltage(voltage);
  }
  public void setVoltage(double forSpinVolt, double backSpinVolt){
    setBackSpinVoltage(backSpinVolt);
    setForwardSpinVoltage(forSpinVolt);
  }

   

  public void setVelocity(double targetVel, double targetAcc){
   
      backSpinMotor.set((backSpinFF.calculate(targetVel, targetAcc) + backSpin.calculate(getVelocityBackSpin(), targetVel)));
      leftFrontSpinMotor.set((frontSpinFF.calculate(targetVel, targetAcc) + frontSpin.calculate(getVelocityFrontSpin(), targetVel)));

    
    //Hi My name is Krish
    //Hi Krish my name is NameNotFoundException
  }

  //Can set distances as enums as well
  //current unit for distance is in inches
  public void shootToPos(double distance){
    double targetVelocity = -0.00166 * distance * distance + 0.449 * distance - 4.346;
    if(distance == 0){
      setVelocity(0, 0);
    }else{
      setVelocity(targetVelocity, 0);
    }
    //double targetVelocity = 0.2052887 * distance + 3.3682;
  }
 
  
  
  public double getVelocityFrontSpin(){
      return frontSpinEncoder.getVelocity();
    //return rightFrontSpinMotor.getEncoder().getVelocity();
  }

  public double getVelocityBackSpin(){
    return backSpinEncoder.getVelocity();
    //return backSpinMotor.getEncoder().getVelocity();
  }

  enum ShooterPos{
    CLOSE,
    MEDIUM,
    FAR
  }
 
  public void changeShootPos(ShooterPos pos){
    shootPos = pos;
  }

public ShooterPos getShootPos(){
    return shootPos;
}

public double shoot(){
  if(shootPos == ShooterPos.CLOSE){
    return closeShootRpm;
  }else if(shootPos == ShooterPos.MEDIUM){
    return mediumShootRpm;
  }else if(shootPos == ShooterPos.FAR){
    return farShootRpm;
  }else{
    //do nothing
  }
  return 0;

}
  
  
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("FrontSpinVel", getVelocityFrontSpin());
    SmartDashboard.putNumber("BackSpinVel", getVelocityBackSpin()); 
  }
}
