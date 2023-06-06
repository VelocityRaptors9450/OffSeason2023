// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ParallelLinkageTurnSubsystem extends SubsystemBase {
  /** Creates a new TestingSubsystemforParallelLinkage. */
  public final CANSparkMax linkageTurn;
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
  private double rotLowerBound;
  private double rotUpperBound;

  public ParallelLinkageTurnSubsystem(int deviceId, double rotLowerBound, double rotUpperBound) {
    this.rotLowerBound = rotLowerBound;
    this.rotUpperBound = rotUpperBound;
    linkageTurn = new CANSparkMax(deviceId, MotorType.kBrushless); // id was 4
    linkageTurn.setIdleMode(IdleMode.kBrake);
  }

  public void run(double power){
    linkageTurn.set(power);
  }

  PIDController d = new PIDController(oldTime, error, derivative);

  //telemetry => for the neo 550 (small motor), the raw
  // .getEncoder().getPosition(); value returns the number
  // of revolutions/rotations experienced by the the faster 
  // turning side; the one that requires less torque to spin
  public double getPosition() {    
    return linkageTurn.getEncoder().getPosition();
  }
  public void resetPosition() {
    linkageTurn.getEncoder().setPosition(0);
  }

  //works with setDefaultCommand, but will need updating for w/ button press
  public void runWithTime(double power) {
    if (g.get() < 0.1) {
      linkageTurn.set(power);
    } else {
      linkageTurn.set(0);
    }

  }

  

  //basic runToPosition method
  public void basicRunToPosition(double position, double power) {
    if (Math.abs(position - getPosition()) > 0) {
      linkageTurn.set(power);
    } else {
      linkageTurn.stopMotor();
    }
  }
  
  public void setErrorValue(double target) {
    this.error = target;
  }
  public void runWithPD(double rotations) {
    // if (error == 0) {
    //   linkageTurn.set(PDWriting(rotations));
    // } else if (Math.abs(error) > 0.2) {
    //   linkageTurn.set(PDWriting(rotations));
    // } else {
    //   linkageTurn.stopMotor();
    // }
    linkageTurn.set(PDWriting(rotations));

  }

  // sets power to motor
  public void testRunWithPD(double target) {
    // if this works the way it should, delete this and the line below and uncomment below
    
    /* 
    linkageTurn.set(TestPDWriting(target)); 
    */

    double currentPos = getPosition();
    System.out.println("CURRENT POSITION: " + currentPos);
    

    

    // stops motor if the position of the linkageTurn gets out of the domain of -4 and 4
    if (currentPos < rotUpperBound && currentPos > rotLowerBound) {
      linkageTurn.set(TestPDWriting(target));
      System.out.println("SeCoNd POSITION:" + currentPos);
    } else {
      System.out.println("STOPPED");
      linkageTurn.stopMotor();
    }
    
    

  }

  public void stopMotor() {
    linkageTurn.stopMotor();
  }

  
 

  /*
     -14.619056701660156 -54  --> 3.6938 degrees/revolution
      14.92857551574707 -53  --> 3.550238 degrees/revolution
      avg is 3.63537 degrees/revolution
   */
  

  // test PID, change powers and condition in if statement to limit power
  public double TestPDWriting(double target) {
    if (restartTimer) {
      g.reset();
      restartTimer = false;
    }
    
    error = target - linkageTurn.getEncoder().getPosition();
    changeInTime = g.get() - oldTime;
    pdPower = error * proportion + 
    ((error - priorError) / (changeInTime)) * derivative;
    
    oldTime = g.get();
    
    if (pdPower > 0.3) {
      pdPower = 0.3;
    } else if (pdPower < -0.3) {
      pdPower = -0.3;
    } 
  
  
    //System.out.println("POWER: " + pdPower + "Proportion: " + error * proportion + "  dv/dt: " + ((error - priorError) / (changeInTime)) * derivative);
    priorError = error;

    
    return pdPower; 
  }
  
  
  double changeInTime = 0.0;
  public double PDWriting(double target) {
    if (restartTimer) {
      g.reset();
      restartTimer = false;
    }
    
    error = target - linkageTurn.getEncoder().getPosition();
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
