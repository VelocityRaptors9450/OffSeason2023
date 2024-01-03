package frc.robot.commands;

import java.util.concurrent.SubmissionPublisher;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ExtensionSetTargetCommand extends CommandBase{
    private final ArmSubsystem arm;
    private double extensionTarget;
    
    

    public ExtensionSetTargetCommand(ArmSubsystem arm, double extensionTarget){
        this.arm = arm;
        addRequirements(arm);
        this.extensionTarget = extensionTarget;
    }


    @Override
    public void initialize(){
        arm.setExtensionGoal(extensionTarget);
        
    
    }

    @Override
    public void execute(){


        

        
    
    }



        
        
        

        
        

    
    @Override
    public void end(boolean interrupted){
       
        
    }

    @Override
    public boolean isFinished(){
        return true;   
    }
}
