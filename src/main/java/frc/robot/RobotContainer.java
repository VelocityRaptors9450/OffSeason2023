// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.PneumaticsSubsytem;
import frc.robot.commands.PneumaticsCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer
{
  PneumaticsSubsytem subsystem = new PneumaticsSubsytem();

  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);


  public RobotContainer()
  {
    subsystem.setDefaultCommand
    (
      new PneumaticsCommand(subsystem, driverController)
    );
    
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() { return null; } 
}