// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class TankDrive extends CommandBase
{

  /*
   * Calls in the RomiDrivetrain and CommandXboxController classes
   * and gives them the custom names "drive" and "controller".
   */
  private final RomiDrivetrain drive;
  private final CommandXboxController controller;


  /* Creates a TankDrive object */
  public TankDrive(RomiDrivetrain drive, CommandXboxController controller)
  {

    // Sets the private variables here to the ones provided in RobotContainer.
    this.drive = drive;
    this.controller = controller;

    // Won't run this command if other commands are currently using the subsytem.
    addRequirements(drive);
    
  }


  /* ---------------------------------------------------------------------------- */


  /* Runs this method first and only once when the command is called. */
  @Override
  public void initialize()
  {

  }


  /* Runs this method every 20 milliseconds while the command is active. */
  @Override
  public void execute()
    {
      // Sets the controller X and Y values to that of the left stick.
      double controllerX = controller.getLeftX(), controllerY = controller.getLeftY();

      // If the controller values are below the deadzone, then they are set to 0.
      if (Math.abs(controllerX) < Constants.controllerDeadzone)
      {
        controllerX = 0;
      }
      if (Math.abs(controllerY) < Constants.controllerDeadzone)
      {
        controllerY = 0;
      }
      

      /*
       * Calls in the "tankdrive" method from RomiDrivetrain, (or "drive"),
       * and provides it with the controller left stick values.
       */
      drive.move(controllerX /3, controllerY /2);
    }


  /* Runs this method when the command ends. */
  @Override
  public void end(boolean interrupted)
  {

  }


  /* If it returns "true", then the command stops. */
  @Override
  public boolean isFinished()
  {
    return false;
  }
}