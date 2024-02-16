// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class EncoderDrive extends CommandBase
{

  /* Calls in the RomiDrivetrain class and names it "drive". */
  private final RomiDrivetrain drive;


  /* Creates a EncoderDrive object */
  public EncoderDrive(RomiDrivetrain drive)
  {

    // Sets the private "drive" variable here to the one provided in RobotContainer.
    this.drive = drive;
    
  }


  /* ---------------------------------------------------------------------------- */


  /* Runs this method first and only once when the command is called. */
  @Override
  public void initialize()
  {
    drive.resetEncoders();

    finished = false;
  }


  /* Runs this method every 20 milliseconds while the command is active. */
  @Override
  public void execute()
  {

    // If the left motor hasn't travelled 6 inches yet, then the robot will continue moving.
    if (drive.getLeftDistanceInch() > -6)
    {
      drive.move(0, -0.5);
    }
    
    // Once the left motor passes 6 inches, then the command will end.
    else
    {
      finished = true;
    }
  }


  /* Runs this method when the command ends. */
  @Override
  public void end(boolean interrupted)
  {
    
  }


  // Creates a boolean that will end the command if it is true.
  private boolean finished = false;

  /* If it returns "true", then the command stops. */
  @Override
  public boolean isFinished()
  {
    return finished;
  }
}
