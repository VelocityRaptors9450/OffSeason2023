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
  public final CANSparkMax linkageTurn1;
  //public final CANSparkMax linkageTurn2;
  public final Timer g = new Timer();
  
  private double error = 0.0;
  private static double priorError = 0.0;
  private static double proportion = 0.018;//0.02 for 1 motor
  private static double derivative = 0.01;//0.018  for 1 motor
  private static double pdPower = 0.0;  
  private static double oldTime = 0.0;
  public static double velocity = 10;
  private static double time = 0;
  private boolean restartTimer = true;
  private double rotLowerBound;
  private double rotUpperBound;

  //used for autnonmous/PID
  public ParallelLinkageTurnSubsystem(int deviceId1, boolean inverted1, int deviceId2, boolean inverted2, double rotLowerBound, double rotUpperBound) {
    this.rotLowerBound = rotLowerBound;
    this.rotUpperBound = rotUpperBound;
    linkageTurn1 = new CANSparkMax(deviceId1, MotorType.kBrushless); // id was 4
    //linkageTurn2 = new CANSparkMax(deviceId2, MotorType.kBrushless);

    linkageTurn1.setIdleMode(IdleMode.kBrake);
    //linkageTurn2.setIdleMode(IdleMode.kBrake);

    linkageTurn1.setInverted(inverted1);
    //linkageTurn2.setInverted(inverted2);
  }
  // used for manual
  public ParallelLinkageTurnSubsystem(int deviceId){
    linkageTurn1 = new CANSparkMax(deviceId, MotorType.kBrushless);
  }


  public void run(double power){
    linkageTurn1.set(power);
    //linkageTurn2.set(power);
  }

  PIDController d = new PIDController(oldTime, error, derivative);

  //telemetry => for the neo 550 (small motor), the raw
  // .getEncoder().getPosition(); value returns the number
  // of revolutions/rotations experienced by the the faster 
  // turning side; the one that requires less torque to spin
  public void setPowerLimited(double power) {
    if (position() > 47) {
        System.out.println("Current Pos: " + position());
        if (power > 0) {
            linkageTurn1.set(0);
            
        } else {
            linkageTurn1.set(power);
        }
    } else if (position() < 2.5) { // means initial position next time will be farther out (by 2.5)
        if (power < 0) {
            linkageTurn1.set(0);
            
        } else {
            linkageTurn1.set(power);
        }
    } else {
        linkageTurn1.set(power);
    }
    
    
    
}
public void setPower(double power){
  linkageTurn1.set(power);
}
public double position(){
  return linkageTurn1.getEncoder().getPosition();
}
  public double getPosition() {    
    return linkageTurn1.getEncoder().getPosition();
  }
  public void resetPosition() {
    linkageTurn1.getEncoder().setPosition(0);
  }

  //works with setDefaultCommand, but will need updating for w/ button press
  public void runWithTime(double power) {
    if (g.get() < 0.1) {
      linkageTurn1.set(power);
     // linkageTurn2.set(power);
    } else {
      linkageTurn1.set(0);
      //linkageTurn2.set(0);
    }

  }

  

  //basic runToPosition method
  public void basicRunToPosition(double position, double power) {
    if (Math.abs(position - getPosition()) > 0) {
      linkageTurn1.set(power);
      //linkageTurn2.set(power);
    } else {
      linkageTurn1.stopMotor();
      //linkageTurn2.stopMotor();
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
    double pow = PDWriting(rotations);
    linkageTurn1.set(pow);
    //linkageTurn2.set(pow);

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
      double pow = TestPDWriting(target) * Math.abs(Math.cos(0));
      linkageTurn1.set(pow);
      //linkageTurn2.set(pow);
      System.out.println("SeCoNd POSITION:" + currentPos);
    } else {
      System.out.println("STOPPED");
      linkageTurn1.stopMotor();
      //linkageTurn2.stopMotor();
      //linkageTurn1.set(-0.03);
      //linkageTurn2.set(-0.03);
    }
    
    

  }

  public void stopMotor() {
    linkageTurn1.stopMotor();
    //linkageTurn2.stopMotor();
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
    
    error = target - linkageTurn1.getEncoder().getPosition();
    changeInTime = g.get() - oldTime;
    pdPower = error * proportion + 
    ((error - priorError) / (changeInTime)) * derivative;
    
    oldTime = g.get();
    
    if (pdPower > 0.2) {
      pdPower = 0.2;
    } else if (pdPower < -0.2) {
      pdPower = -0.2;
    } 
  
  
    //System.out.println("POWER: " + pdPower + "Proportion: " + error * proportion + "  dv/dt: " + ((error - priorError) / (changeInTime)) * derivative);
    priorError = error;

    System.out.println("Power: "+ pdPower);
    return pdPower; 
  }
  
  
  double changeInTime = 0.0;
  public double PDWriting(double target) {
    if (restartTimer) {
      g.reset();
      restartTimer = false;
    }
    
    error = target - linkageTurn1.getEncoder().getPosition();
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
