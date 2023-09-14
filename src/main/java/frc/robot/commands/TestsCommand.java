// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.TestsSubsystem;

public class TestsCommand extends CommandBase {
  /** Creates a new Tests. */
  private CommandXboxController controller;
  private TestsSubsystem motorTest;
  public TestsCommand(TestsSubsystem motorTest, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.controller = controller;
    this.motorTest = motorTest;
    addRequirements(motorTest);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = controller.getLeftY();
    double rotation = controller.getRightY();

    motorTest.speedPower(speed);
    motorTest.speedPower(rotation);
  }

 
}
