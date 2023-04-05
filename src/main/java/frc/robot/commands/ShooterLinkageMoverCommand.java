// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterLinkageMoverSubsystem;

public class ShooterLinkageMoverCommand extends CommandBase {
  /** Creates a new ShooterLinkageMoverCommand. */
  ShooterLinkageMoverSubsystem linkage;
  
  public ShooterLinkageMoverCommand(ShooterLinkageMoverSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.linkage = subsystem;
    addRequirements(subsystem);
    linkage.turn.getEncoder().setPosition(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    linkage.rampTimer.restart();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // target = number of revolutions
    linkage.toggle(0.5);
    System.out.println("linkage toggled");
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
