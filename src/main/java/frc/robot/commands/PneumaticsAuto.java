// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.PneumaticsSubsystem;

public class PneumaticsAuto extends CommandBase
{
  PneumaticsSubsystem subsystem;
  CommandXboxController controller;
  int rate;

  /** Creates a new PneumaticsAuto. */
  public PneumaticsAuto(PneumaticsSubsystem subsystem, CommandXboxController controller, int rate)
  {
    this.subsystem = subsystem;
    this.controller = controller;
    this.rate = rate;

    addRequirements(subsystem);
  }

  @Override
  public void initialize()
  {
    subsystem.disableCompressor();
  }

  Timer time = new Timer();
  boolean lineSwitcher = true;
  @Override
  public void execute()
  {
    if (lineSwitcher)
      subsystem.enableCompressor();
    else
      subsystem.disableCompressor();
    
    if (time.hasElapsed(rate/2.0))
      lineSwitcher = !lineSwitcher;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
