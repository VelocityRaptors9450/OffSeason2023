// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.PneumaticsSubsytem;

public class PneumaticsCommand extends CommandBase
{
  PneumaticsSubsytem subsystem;
  CommandXboxController controller;

  public PneumaticsCommand(PneumaticsSubsytem subsystem, CommandXboxController controller) {
    this.subsystem = subsystem;
    this.controller = controller;

    addRequirements(subsystem);
  } 

  @Override
  public void initialize() {}

  @Override
  public void execute()
  {
    if (controller.getHID().getYButtonPressed())
    { subsystem.enableCompressor(); }

    else if (controller.getHID().getAButtonPressed())
    { subsystem.disableCompressor(); }

    if (controller.getHID().getBButtonPressed()) {
      subsystem.extend();
    } else if (controller.getHID().getXButtonPressed()) {
      subsystem.retract();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() { return false; }
}
