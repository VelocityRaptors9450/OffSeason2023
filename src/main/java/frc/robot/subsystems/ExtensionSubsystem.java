// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExtensionSubsystem extends SubsystemBase {
  /** Creates a new ExtensionSubsystem. */
  public ExtensionSubsystem() {}
  
  public CANSparkMax extensionMovement = new CANSparkMax(18, MotorType.kBrushless);
  public Timer t = new Timer();

    private static double error = 0.0;
  private static double priorError = 0.0;
  private static double proportion = 0.09;
  private static double derivative = 0.000012;
  private static double pdPower = 0.0;  
  private static double timeChange = 0.02;
  public static double velocity = 10;
  private static double time = 0;
  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void moveExtension(double distance) {
    extensionMovement.set(PDPowering(distance));
  } 

  public void rotateHalfSecond() {
    if (t.get() < 0.25) {
      extensionMovement.set(0.2);
   } else {
    extensionMovement.set(0);
   }

    

  }

  public void move(double power) {
    
    extensionMovement.set(power > 1 ? 1 : power);

  }

  public void stop() {
    extensionMovement.set(0);
  }

  public void testMovement() {
    extensionMovement.set(0.2);
  }

  public double PDPowering(double target) {
    timeChange = t.get();
    error = target - extensionMovement.getEncoder().getPosition();
    pdPower = error * proportion + 
    ((error - priorError) / (t.get() - timeChange)) * derivative;
    
    if (pdPower > 0.3) {
      pdPower = 0.3;
    } else if (pdPower < -0.3) {
      pdPower = -0.3;
    } else if (pdPower < 0.05) {
      //pdPower += (0.1 - 0.1/(1+Math.pow(Math.E, -0.5*(extensionMovement.getEncoder().getPosition()-19))));
    }
  
  
    System.out.println("POWER: " + pdPower + "  dv/dt: " + ((error - priorError) / (t.get() - timeChange)) * derivative);
    priorError = error;
    t.restart();
    return pdPower; 
}
  

}
