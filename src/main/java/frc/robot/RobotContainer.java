package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer
{
  private final DriveTrain drive = new DriveTrain();

  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);


  public RobotContainer()
  {
    drive.setDefaultCommand( new DriveCommand(drive, driverController) ); // Runs the DriveCommand in TeleOp.

    configureBindings();
  }

  
  private void configureBindings() {}

  public Command getAutonomousCommand()
  {
    return null;
  }
}