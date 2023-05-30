// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.ExampleCommand;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    // motor6.getEncoder().setPosition(0);
    //intakeLeft.getEncoder().setPosition(0);
    //intakeRight.getEncoder().setPosition(0);

    //refillSolenoid.set(false);
  }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase  exampleMethodCommand() {
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

  /* 
  public static CANSparkMax motor6 = new CANSparkMax(1, MotorType.kBrushless);
  public static CANSparkMax motor4 = new CANSparkMax(2, MotorType.kBrushless);

  private CANSparkMax leftMotor1 = new CANSparkMax(6, MotorType.kBrushless);
  //private CANSparkMax leftMotor2 = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax rightMotor1 = new CANSparkMax(4, MotorType.kBrushless);
  //private CANSparkMax rightMotor2 = new CANSparkMax(4, MotorType.kBrushless);

  private Joystick joy1 = new Joystick(0);

  // public static CANSparkMax motor6 = new CANSparkMax(5, MotorType.kBrushless);
  public static CANSparkMax intakeRight = new CANSparkMax(6, MotorType.kBrushless);
  public static CANSparkMax intakeLeft = new CANSparkMax(4, MotorType.kBrushless);


  // solenoid not connected to air thing
  Solenoid testingSolenoid_PH = new Solenoid(7, PneumaticsModuleType.REVPH, 7);
  Solenoid refillSolenoid = new Solenoid(7, PneumaticsModuleType.REVPH, 6);

  Compressor pcmCompressor = new Compressor(7, PneumaticsModuleType.CTREPCM);
  

  // private double startPos = motor6.getEncoder().getPosition();
  public static Timer t = new Timer();
  public static Timer l = new Timer();
  public static Timer g = new Timer();
  Timer quit = new Timer();


  public static enum IntakeStep {
    INITIAL_RAMP_UP,
    INITIAL, // rampUpToggle boolean equivalent
    SLOW_INTAKE,
    IS_OUTPUTTING,
    DONE_OUTPUTTING,
  }

  private IntakeStep intakeStep = IntakeStep.INITIAL;
  
  private static double error = 0.0;
  private static double priorError = 0.0;
  private static double proportion = 0.09;
  private static double derivative = 0.000012;
  private static double pdPower = 0.0;  
  private static double timeChange = 0.02;
  private static double oldpos = 0;
  private static boolean temp = true;
  private static boolean isOutput = false;
  private static boolean initial = true;
  private static boolean rampUPToggle = true;
  public static double velocity = 10;
  private static double time = 0;
  
*/
  @Override
  public void periodic() {
    //1:18
    //42 tics per rev
    // This method will be called once per scheduler run

     //motorRunning();

    //intake(true, 0.4);


    //motor4.set(0.1);
    //motor6.set(-0.05);
    //motor4.set(0.05);
    // set when velocity is lower lower power out put on intake
     /*
      * 
      
      */
    //get position return number of rotations
    


    //rightMotor2.set(-right);

    //System.out.println(motor2.get());

    //motorRunning(18);
     /*
      * 
      FIGURE OUT HOW TO RUN TWO THINGS.....
      */
    //get position return number of rotations
    

    

    //pneumatics();



  }
  public double rampUp(double time, double powerTo) {
    return 2 * powerTo / (1 + Math.pow(Math.E, -9*(time))) - 0.4;

  }
  public double rampDown(double time, double powerFrom) {
    return -(2 * powerFrom / (1 + Math.pow(Math.E, -9*(time - 1))) - 0.4);

  }

/*
  public void intake(boolean in, double power) {
    //System.out.println("Velocity Left: " + Math.round(intakeLeft.getEncoder().getVelocity()) + "  Velocity Right: " + Math.round(intakeLeft.getEncoder().getVelocity()));
    
    
    // timeChange = g.get();
    // g.reset();
    double changeInPos = (Math.abs(intakeRight.getEncoder().getPosition()) + Math.abs(intakeLeft.getEncoder().getPosition())) / 2 - oldpos;
    velocity = changeInPos / 0.02;
    System.out.println("Velocity: " + velocity);
    oldpos = (Math.abs(intakeRight.getEncoder().getPosition()) + Math.abs(intakeLeft.getEncoder().getPosition())) / 2;
    
    // approximately 19 revolutions / second
    
    if (velocity < 70 && initial) {
      if (rampUPToggle) {
        // assigns current state
        intakeStep = IntakeStep.INITIAL_RAMP_UP;
       
        t.reset();
        rampUPToggle = false;
      }
      

      if (rampUp(t.get(), 0.4) > 0.4) {
        intakeLeft.set(power);
        intakeRight.set(-power);
      } else {
        System.out.println("------------------- \n RAMPING UP \n ---------------------------");

        intakeLeft.set(rampUp(t.get(), 0.4));
        intakeRight.set(-rampUp(t.get(), 0.4));
      }
      
    } else if (!isOutput && velocity > 70) { // intake 
      intakeStep = IntakeStep.INITIAL;
      initial = false;
      intakeLeft.set(power);
      intakeRight.set(-power);
    } else if (!isOutput){
      if (temp) {
        g.reset(); 

        temp = false;
        System.out.println("------------------- \n SLOW SPEED \n ---------------------------");
        intakeStep = IntakeStep.SLOW_INTAKE;
        intakeLeft.set(0.04);
        intakeRight.set(-0.04);
        
      }
      if (g.get() >= 2.0) {
        System.out.println("------------------- \n OUTPUTTING \n ---------------------------");
        intakeStep = IntakeStep.IS_OUTPUTTING;
        intakeLeft.set(-0.4);
        intakeRight.set(0.4);
            
        
      } 
      if (g.get() >= 3.5) {
        
        isOutput = true;
        g.reset();
        t.reset();
      }
    }

     if (rampDown(t.get(), 0.4) > 0 && t.get() <= 1 && isOutput) {
      intakeStep = IntakeStep.DONE_OUTPUTTING;
      System.out.println("------------------- \n RAMPING DOWN, t = " + t.get() + "\n ---------------------------");
      intakeLeft.set(-rampDown(t.get(), 0.4));
      intakeRight.set(rampDown(t.get(), 0.4));
    } else if (t.get() > 2 && g.get() > 2 && isOutput) {
      intakeLeft.set(0);
      intakeRight.set(0);

      temp = true;
      isOutput = false;
      initial = true;
      rampUPToggle = true;
      velocity = 0;
    }
    
    
    
  }
*/
  public void pneumatics() {
    // 
    /* 
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
    */

    // if compressor disabled, then pcmCompressor.enabled();
    // max pressure 60, min pressure 50
    //System.out.println(pcmCompressor.getPressure());
    /* 
    
    if (pcmCompressor.getPressure() <= 50) {
      refillSolenoid.set(true);
    } else {
      refillSolenoid.set(false);

    }



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
    
    
    
  
   */
}
/* 
  public void motorRunning(double i) {
    System.out.println(motor6.getEncoder().getPosition());
    
        if(i-motor6.getEncoder().getPosition() > 0 ){
          motor6.set(PDWriting(18)); 
        }else {
          motor6.set(-0.3);
        }   
   
     
  }

  
  
  //test commit - raghav
  public double PDWriting(double target) {
    timeChange = g.get();
    error = target - motor6.getEncoder().getPosition();
    pdPower = error * proportion + 
    ((error - priorError) / (g.get() - timeChange)) * derivative;
    
    if (pdPower > 0.3) {
      pdPower = 0.3;
    } else if (pdPower < -0.3) {
      pdPower = -0.3;
    } else if (pdPower < 0.05) {
      pdPower += (0.1 - 0.1/(1+Math.pow(Math.E, -0.5*(motor6.getEncoder().getPosition()-19))));
    }
   
   
    System.out.println("POWER: " + pdPower + "  dv/dt: " + ((error - priorError) / (g.get() - timeChange)) * derivative);
    priorError = error;
    g.restart();
    return pdPower; 
  }
  */
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
