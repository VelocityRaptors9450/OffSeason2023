// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.EncoderDrive;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final RomiDrivetrain drive = new RomiDrivetrain();
  private final CommandXboxController controller = new CommandXboxController(0);

  

  /* This is where the subsystems and commands will go to be run. */
  public RobotContainer()
  {

    // Makes the TankDrive command run by default.
    drive.setDefaultCommand( new TankDrive(drive, controller) );


    // Runs the DriveEncoder command when the "A" button is pressed.
    controller.a().onTrue( new EncoderDrive(drive) );

  }
}
