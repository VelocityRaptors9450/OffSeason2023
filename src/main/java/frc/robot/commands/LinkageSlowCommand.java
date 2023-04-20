package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class LinkageSlowCommand extends CommandBase{

    private ShooterSubsystem shooter;
    private double slope, startPower, targetPower, seconds;
    private boolean isDone = false;
    Timer time = new Timer();
    


    public LinkageSlowCommand(ShooterSubsystem shooter, double targetPower, double seconds){
        this.shooter = shooter;
        this.targetPower = targetPower;
        this.seconds = seconds;
        


        addRequirements(shooter);
    }

//Called when the command is initially scheduled.
  @Override
  public void initialize() {

    startPower = shooter.getPower();
    slope = (targetPower - startPower) / (seconds);


    isDone = false;
    time.start();
    time.reset();

    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double power = slope * time.get() + startPower;
    if(power > 1){
      power = 1;
    }
    if(power < -1){
      power = -1;
    }

    //System.out.println("startPower: " + startPower);
    //System.out.println("time elapsed: " + time.get());
    //System.out.println("slope: " + slope);

    //System.out.println("power: " + power);

    shooter.setPower(power);

    if(time.get() >= seconds){
      isDone = true;
    }

    // System.out.println("getPower: " + shooter.getPower());
    // System.out.println("motor1 Power: " + shooter.getMotor1Power());
    // System.out.println("motor2 Power: " + shooter.getMotor2Power());


    

    

   
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return isDone;
  }

}