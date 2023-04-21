package frc.robot.commands;

import java.util.function.Supplier;

import org.ejml.equation.ManagerFunctions.Input1;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtensionSubsystem;

public class ExtensionCommand extends CommandBase{

    ExtensionSubsystem extension;
    Supplier<Double> inPower, outPower;
    


    public ExtensionCommand(ExtensionSubsystem extension, Supplier<Double> outPower, Supplier<Double> inPower){
        this.extension = extension;
        this.inPower = inPower;
        this.outPower = outPower;

        addRequirements(extension);
    }

//Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double inPowerAmt = -2 * inPower.get() / 3;
    double outPowerAmt =2 *  outPower.get() / 3;
    double totalPowerAmt = inPowerAmt + outPowerAmt;

    if(Math.abs(totalPowerAmt) > 0.05){
      extension.setPower(totalPowerAmt);
    }else{
      extension.setPower(0);
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