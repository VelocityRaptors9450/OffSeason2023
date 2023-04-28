// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TestingSubsystemforParallelLinkage;

public class ParallelLinkage extends CommandBase {
  /** Creates a new ParallelLinkage. */
  double time;
  boolean up;
  double amountPos = 0;
  double amountNeg = 0;

  boolean returnToZero = false;
  Timer t = new Timer();
  TestingSubsystemforParallelLinkage test;
  public ParallelLinkage(TestingSubsystemforParallelLinkage test, boolean up) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.test = test;
    this.up = up;
    returnToZero = false;

    addRequirements(test);
  }

  public ParallelLinkage(TestingSubsystemforParallelLinkage test) {
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
    test.setErrorValue(15);
    amountPos = test.getPosition() + 5;
    amountNeg = test.getPosition() - 5;

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
      if (amountPos < Constants.maxWrist)
        test.testRunWithPD(amountPos);
    } else if (returnToZero) {
      test.testRunWithPD(0);
    } else {
      if (amountNeg > Constants.minWrist)
        test.testRunWithPD(amountNeg);
    } 
    //test.runWithPD(0);
    System.out.println(test.getPosition());
   
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
