package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootingCommand extends CommandBase{

    ShooterSubsystem shooter;
    Supplier<Double> power;
    


    public ShootingCommand(ShooterSubsystem shooter, Supplier<Double> power){
        this.shooter = shooter;
        this.power = power;

        addRequirements(shooter);
    }

//Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
    double realLinkagePower = power.get() / 4;
    

    shooter.setLinkagePower(realLinkagePower);
    
    */

   
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }

}