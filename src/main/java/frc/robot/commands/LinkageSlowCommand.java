package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class LinkageSlowCommand extends CommandBase{

    ShooterSubsystem shooter;
    boolean go = true;
    double power;
    


    public LinkageSlowCommand(ShooterSubsystem shooter, boolean go, double power){
        this.shooter = shooter;
        this.go = go;
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
    if(go){
        shooter.setLinkagePower(0.25);
        shooter.setPower(power);

    }else{
        shooter.setLinkagePower(0);
        shooter.setPower(0);
    }

    

   
    
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