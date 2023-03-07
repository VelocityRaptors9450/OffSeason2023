// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ExampleCommand;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    motor6.getEncoder().setPosition(0);
    refillSolenoid.set(false);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */

        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
  public static CANSparkMax motor6 = new CANSparkMax(6, MotorType.kBrushless);
  // solenoid not connected to air thing
  Solenoid testingSolenoid_PH = new Solenoid(7, PneumaticsModuleType.REVPH, 7);
  Solenoid refillSolenoid = new Solenoid(7, PneumaticsModuleType.REVPH, 6);

  Compressor pcmCompressor = new Compressor(7, PneumaticsModuleType.CTREPCM);
  

  private double startPos = motor6.getEncoder().getPosition();
  public static Timer t = new Timer();
  public static Timer l = new Timer();
  public static Timer g = new Timer();


  private static double error = 0.0;
  private static double priorError = 0.0;
  private static double proportion = 0.05;
  private static double derivative = 0.0;
  private static double pdPower = 0.0;
  private static double timeChange = 0.0;
  @Override
  public void periodic() {

    // This method will be called once per scheduler run
     //motorRunning();
    
    

    
    //System.out.println(motor2.get());
    
    //pneumatics();



  }

  public void pneumatics() {
    // 
    if(testingSolenoid_PH.get() == false){
      if(t.get() >= 1.0){
        testingSolenoid_PH.set(true);
        t.restart();
      }
    }else{
      if(t.get() >= 1.0){
        testingSolenoid_PH.set(false);
        t.restart();
      }
    }

    // if compressor disabled, then pcmCompressor.enabled();
    // max pressure 60, min pressure 50
    System.out.println(pcmCompressor.getPressure());
    /* 
    
    if (pcmCompressor.getPressure() <= 50) {
      refillSolenoid.set(true);
    } else {
      refillSolenoid.set(false);

    }
*/

    if(refillSolenoid.get() == false){
      if(l.get() >= 5.0){
        refillSolenoid.set(true);
        l.restart();
      }
    }else{
      if(l.get() >= 1.0){
        refillSolenoid.set(false);
        l.restart();
      }
    }
    
    
    
  }

  public void motorRunning() {
    System.out.println(motor6.getEncoder().getPosition());

    if(20-motor6.getEncoder().getPosition() > 0){
        motor6.set(PDWriting(20));  
    }else {
      motor6.set(0);
    }
  }
  
  //test commit - raghav
  public double PDWriting(int target) {
    timeChange = g.get();
    error = Math.abs(motor6.getEncoder().getPosition() - target);
    pdPower = error * proportion + (error - priorError) / timeChange * derivative;
    
    if (pdPower > 0.3) {
      pdPower = 0.3;
    }

    priorError = error;
    g.restart();
    return pdPower; 
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
