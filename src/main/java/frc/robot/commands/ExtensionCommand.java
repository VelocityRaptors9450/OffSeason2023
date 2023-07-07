package frc.robot.commands;

import java.util.function.Supplier;

import org.ejml.equation.ManagerFunctions.Input1;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtensionSubsystem;

public class ExtensionCommand extends CommandBase{

    ExtensionSubsystem extension;
    Supplier<Double> inPower, outPower;
    Supplier<Boolean> leftBumperPressed, rightBumperPressed;
    double startingPosition, target;
    


    public ExtensionCommand(ExtensionSubsystem extension, Supplier<Double> outPower, Supplier<Double> inPower, Supplier<Boolean> leftBumperPressed, Supplier<Boolean> rightBumperPressed){
        this.extension = extension;
        this.inPower = inPower;
        this.outPower = outPower;
        this.leftBumperPressed = leftBumperPressed;
        this.rightBumperPressed = rightBumperPressed;

        addRequirements(extension);
    }

//Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingPosition = extension.position();
    target = startingPosition;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double inPowerAmt = -2 * inPower.get() / 3;
    double outPowerAmt =2 *  outPower.get() / 3;
    double totalPowerAmt = inPowerAmt + outPowerAmt;
    boolean leftBumper = leftBumperPressed.get();
    boolean rightBumper = rightBumperPressed.get();

    if(leftBumper){
      target = startingPosition;
    }else if(rightBumper){
      target = startingPosition + 47;
    }

    if(Math.abs(totalPowerAmt) > 0.05){
      extension.setPower(totalPowerAmt);
      target = extension.position();
    }else{
      //extension.pid(target);
      extension.setPower(0);
    }

    System.out.println("Start: " + startingPosition);
    System.out.println("Position: " + extension.position());

   
    
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