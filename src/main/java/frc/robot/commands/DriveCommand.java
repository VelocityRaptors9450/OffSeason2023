package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;

public class DriveCommand extends CommandBase
{
  DriveTrain drive;
  CommandXboxController controller;

  public DriveCommand(DriveTrain drive, CommandXboxController controller)
  {
    this.drive = drive;
    this.controller = controller;
  }


  @Override
  public void initialize() {}

  @Override
  public void execute()
  {
    drive.move(controller.getLeftY(), controller.getRightX()); // Runs the "move" method in DriveTrain with the Controller values.
  }

  @Override
  public void end(boolean interrupted) {}

  
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
