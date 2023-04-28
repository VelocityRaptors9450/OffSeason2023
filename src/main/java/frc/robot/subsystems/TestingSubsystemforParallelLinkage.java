// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TestingSubsystemforParallelLinkage extends SubsystemBase {
  /** Creates a new TestingSubsystemforParallelLinkage. */
  public final CANSparkMax wrist;
  public final Timer g = new Timer();
  
  private double error = 0.0;
  private static double priorError = 0.0;
  private static double proportion = 0.07;
  private static double derivative = 0.00;
  private static double pdPower = 0.0;  
  private static double oldTime = 0.0;
  public static double velocity = 10;
  private static double time = 0;
  private boolean restartTimer = true;

  public TestingSubsystemforParallelLinkage() {
    wrist = new CANSparkMax(4, MotorType.kBrushless);
  }

  public void run(double power){
    wrist.set(power);
  }

  PIDController d = new PIDController(oldTime, error, derivative);

  //telemetry => for the neo 550 (small motor), the raw
  // .getEncoder().getPosition(); value returns the number
  // of revolutions/rotations experienced by the the faster 
  // turning side; the one that requires less torque to spin
  public double getPosition() {
    return wrist.getEncoder().getPosition();
  }
  public void resetPosition() {
    wrist.getEncoder().setPosition(0);
  }

  //works with setDefaultCommand, but will need updating for w/ button press
  public void runWithTime(double power) {
    if (g.get() < 0.1) {
      wrist.set(power);
    } else {
      wrist.set(0);
    }

  }

  

  //basic runToPosition method
  public void basicRunToPosition(double position, double power) {
    if (Math.abs(position - getPosition()) > 0) {
      wrist.set(power);
    } else {
      wrist.stopMotor();
    }
  }
  
  public void setErrorValue(double target) {
    this.error = target;
  }
  public void runWithPD(double rotations) {
    // if (error == 0) {
    //   wrist.set(PDWriting(rotations));
    // } else if (Math.abs(error) > 0.2) {
    //   wrist.set(PDWriting(rotations));
    // } else {
    //   wrist.stopMotor();
    // }
    wrist.set(PDWriting(rotations));

  }

  public void testRunWithPD(double rotations) {
    if (rotations > 0 && getPosition() < Constants.maxWrist) {
      wrist.set(TestPDWriting(rotations));
    }
  }


  /*
     -14.619056701660156 -54  --> 3.6938 degrees/revolution
      14.92857551574707 -53  --> 3.550238 degrees/revolution
      avg is 3.63537 degrees/revolution
   */
  

   
  public double TestPDWriting(double target) {
    if (restartTimer) {
      g.reset();
      restartTimer = false;
    }
    
    error = target - wrist.getEncoder().getPosition();
    changeInTime = g.get() - oldTime;
    pdPower = error * proportion + 
    ((error - priorError) / (changeInTime)) * derivative;
    
    oldTime = g.get();
    
    if (pdPower > 0.05) {
      pdPower = 0.05;
    } else if (pdPower < -0.05) {
      pdPower = -0.05;
    } 
  
  
    System.out.println("POWER: " + pdPower + "Proportion: " + error * proportion + "  dv/dt: " + ((error - priorError) / (changeInTime)) * derivative);
    priorError = error;

    
    return pdPower; 
  }
  
  
  double changeInTime = 0.0;
  public double PDWriting(double target) {
    if (restartTimer) {
      g.reset();
      restartTimer = false;
    }
    
    error = target - wrist.getEncoder().getPosition();
    changeInTime = g.get() - oldTime;
    pdPower = error * proportion + 
    ((error - priorError) / (changeInTime)) * derivative;
    
    oldTime = g.get();
    
    if (pdPower > 0.6) {
      pdPower = 0.6;
    } else if (pdPower < -0.6) {
      pdPower = -0.6;
    } 
  
  
    System.out.println("POWER: " + pdPower + "Proportion: " + error * proportion + "  dv/dt: " + ((error - priorError) / (changeInTime)) * derivative);
    priorError = error;

    
    return pdPower; 
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
