// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
  public ExampleSubsystem() {}

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
  private CANSparkMax motor2 = new CANSparkMax(2, MotorType.kBrushless);
  Solenoid testingSolenoid_PH = new Solenoid(7, PneumaticsModuleType.REVPH, 7);
  private double startPos = motor2.getEncoder().getPosition();
  public static Timer t = new Timer();
  
  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    /* 
    if(42-motor2.getEncoder().getPosition() > 0 ){
        motor2.set(0.2);
    }else{
      motor2.set(0);
    }
    */
    
    //System.out.println(motor2.get());
    
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
  //  testingSolenoid_PH.toggle();
    // try {
    //   wait(5000);
    // } catch (Exception E) {
    //   //System.out.print(E);
    // }
    
    // testingSolenoid_PH.toggle();


  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
