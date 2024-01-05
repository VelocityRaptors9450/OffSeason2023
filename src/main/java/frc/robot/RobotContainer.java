package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer
{
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  public RobotContainer()
  {
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand()
  {
    return null;
  }
}