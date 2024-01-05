// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;


public class ExtensionSubsystem extends SubsystemBase {
  private CANSparkMax extensionMotor = new CANSparkMax(35,MotorType.kBrushless);
  
  ProfiledPIDController pid = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0)); //0.4, 0, 0
  //ElevatorFeedforward elevFF = new ElevatorFeedforward(0.001, , ); // 0.001, 0.035, 0.015

  public ExtensionSubsystem() {
    extensionMotor.setIdleMode(IdleMode.kBrake);
    extensionMotor.getEncoder().setPosition(0);
    pid.reset(getPosition());
  }
  
  //gear ratio 13.5 rotations for 1 full metal rotation
  // 32 rotations for mechanical limit

  public void updateExtensionOutput(){
    double ffValue = calculateExtensionFF();
    double percentOutput = MathUtil.clamp(calculateExtensionPID() + ffValue, -1.0, 1.0);
    double voltage = convertToVolts(percentOutput);

    SmartDashboard.putNumber("Extension FF", ffValue);
    SmartDashboard.putNumber("Extension Voltage", voltage);
    
    /*
    if (Math.abs(voltage) > 4) {
        setVoltage(Math.signum(voltage)*4);
    } else {
        setVoltage(voltage);
    }
    */

    setVoltage(voltage);
  }

  public double calculateExtensionFF() {
      return 0;
  }

  public double calculateExtensionPID() {
    return pid.calculate(getPosition(), pid.getGoal());
  }

  public void setExtensionGoal(double goal) {
    pid.setGoal(goal);
  }


  public void setPower(double power){
    extensionMotor.set(power);
  }

  public void setVoltage(double voltage) {
    extensionMotor.setVoltage(voltage);
  }
  

  public double getPosition(){
    return extensionMotor.getEncoder().getPosition();
  }

  private double convertToVolts(double percentOutput){
    return percentOutput * Robot.getInstance().getVoltage();
  }

  @Override
  public void periodic() {
    //updateExtensionOutput();
    extensionMotor.setVoltage(-0.23);

    //Horizontal -> -0.23
    //Up top -> 0.13
    
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Extension Position", getPosition());
  }
}
