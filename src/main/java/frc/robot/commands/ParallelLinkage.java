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
  Timer t = new Timer();
  TestingSubsystemforParallelLinkage test;
  public ParallelLinkage(TestingSubsystemforParallelLinkage test) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.test = test;
    addRequirements(test);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {time = System.currentTimeMillis(); t.reset();t.restart();}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(/*System.currentTimeMillis() - time*/t.get() < 0.3){
      test.run(0.1);

    } else {
      test.run(0);
    }
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
