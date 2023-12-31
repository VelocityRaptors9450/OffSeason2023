// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ParallelLinkageWristSubsystem;

public class ParallelLinkageWristCommand extends CommandBase {
  /** Creates a new ParallelLinkage. */
  double time;
  boolean up;
  boolean resetPos = true;

  double target = 0;
  boolean returnToZero = false; 
  Timer t = new Timer();
  private static ParallelLinkageWristSubsystem test;
 
  public ParallelLinkageWristCommand(ParallelLinkageWristSubsystem test, boolean up) {
    // Use addRequirements() here to declare subsystem dependencies.
    test.wrist.getEncoder().setPosition(0);

  
    this.test = test;
    this.up = up;
    returnToZero = false;

    addRequirements(test);
  }



  public ParallelLinkageWristCommand(ParallelLinkageWristSubsystem test) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.test = test;
    this.up = true;
    returnToZero = true;

    addRequirements(test);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    test.g.start();
    test.g.reset();
    //test.resetPosition();
    // assigns starting error value for more accurate PID start
    test.setErrorValue(8);
    target = 8;
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  
  @Override
  public void execute() {
    //test.runWithTime(0.1);
    
      /*
     -14.619056701660156 -54  --> 3.6938 degrees/revolution
      14.92857551574707 -53  --> 3.550238 degrees/revolution
      avg is 3.63537 degrees/revolution for the PD
   */

    
    if (!up) {
      // moves to target (on press of y button)
      test.testRunWithPD(-target);
    } else if (returnToZero) {
      // moves to position 0
      test.testRunWithPD(0);
    } else {
      // moves to target (on press of x button)
      test.testRunWithPD(target);
    } 
    //test.runWithPD(0);
    //System.out.println(test.getPosition());
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
