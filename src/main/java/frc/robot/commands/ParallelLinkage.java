// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestingSubsystemforParallelLinkage;

public class ParallelLinkage extends CommandBase {
  /** Creates a new ParallelLinkage. */
  double time;
  boolean up;
  Timer t = new Timer();
  TestingSubsystemforParallelLinkage test;
  public ParallelLinkage(TestingSubsystemforParallelLinkage test, boolean up) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.test = test;
    this.up = up;

    addRequirements(test);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    test.g.start();
    test.g.reset();
    test.resetPosition();
    test.setErrorValue(15);
  }

  // Called every time the scheduler runs while the command is scheduled.
  
  @Override
  public void execute() {
    //test.runWithTime(0.1);
    if (!up) {
      test.runWithPD(15);
    } else {
      test.runWithPD(-15);
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
