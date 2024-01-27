package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterSubsytem;

public class ShooterCommand extends CommandBase
{
  ShooterSubsytem shooter;
  CommandXboxController controller;
  private static boolean finished = false;
  

  public ShooterCommand( ShooterSubsytem shooter, CommandXboxController controller )
  {
    this.shooter = shooter;
    this.controller = controller;
    
    
  }


  @Override
  public void initialize() {}

  @Override
  public void execute()
  {
    shooter.shoot();

    if ( controller.getHID().getBButtonReleased() )
    {
      finished = true;
    }
  }

  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished()
  {
    return finished;
  }
}
