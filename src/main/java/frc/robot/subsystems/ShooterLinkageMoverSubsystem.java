// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterLinkageMoverSubsystem extends SubsystemBase {
  /** Creates a new ShooterLinkageMoverSubsystem. */
  public CANSparkMax turn = new CANSparkMax(4, MotorType.kBrushless);
    

  // private double startPos = motor6.getEncoder().getPosition();
  public static Timer t = new Timer();
  public Timer rampTimer = new Timer();
  public static Timer g = new Timer();
  Timer quit = new Timer();

  private static double error = 0.0;
  private static double priorError = 0.0;
  private static double proportion = 0.024;
  private static double derivative = 0.0;
  private static double pdPower = 0.0;  
  private static double timeChange = 0.02;
  private static double oldpos = 0;
  private static boolean close = true;
  private static boolean isOutput = false;
  private static boolean initial = true;
  private static boolean rampUPToggle = true;
  public static double velocity = 10;
  private static double time = 0;
  
  
  public ShooterLinkageMoverSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void turn() {
    turn.set(0.3);
  }
  public void toggle(double target) {
    
    if (turn.getEncoder().getPosition() > -(target * 75 - 0.5) && close) {
      
      turn.set(-fullLinearRamp(0.5, rampTimer.get(), 0.4));
      System.out.println(rampTimer.get() + "  " + turn.getEncoder().getPosition());
      t.restart();
    } else {
      // hold for quarter second
      if (t.get() > 0.25) {
        close = false;
        turn.set(PDWriting(0));
        if (turn.getEncoder().getPosition() > -(0.5)) {
          close = true;
        }
      }
    }
    
  }

  public double linearRampUp(double timeToRamp, double currentTime, double powerTo) {
    return powerTo * currentTime / timeToRamp;
  }

  public double linearRampDown(double timeToRamp, double currentTime, double powerFrom) {
    return -powerFrom * currentTime / timeToRamp + powerFrom;
  }

  public double fullLinearRamp(double timeToRamp, double currentTime, double powerTo) {
    return -Math.abs(powerTo * (currentTime - (timeToRamp / 2)) / (timeToRamp / 2)) + powerTo;
  }

  //logistic ramp
  public double rampUp(double time, double powerTo) {
    return 2 * powerTo / (1 + Math.pow(Math.E, -9*(time))) - 0.4;

  }
  //logistic ramp
  public double rampDown(double time, double powerFrom) {
    return -(2 * powerFrom / (1 + Math.pow(Math.E, -9*(time - 1))) - 0.4);

  }

    public double PDWriting(double target) {
      timeChange = g.get();
      error = target - turn.getEncoder().getPosition();
      pdPower = error * proportion + 
      ((error - priorError) / (g.get() - timeChange)) * derivative;
      
      if (pdPower > 0.1) {
        pdPower = 0.1;
      } else if (pdPower < -0.1) {
        pdPower = -0.1;
      } 
    
      System.out.println("POWER: " + pdPower + "  dv/dt: " + ((error - priorError) / (g.get() - timeChange)) * derivative);
      priorError = error;
      g.restart();
      return pdPower; 
  }

}
