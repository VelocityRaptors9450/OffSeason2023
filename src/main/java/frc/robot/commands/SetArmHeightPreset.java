package frc.robot.commands;

import java.util.concurrent.SubmissionPublisher;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SetArmHeightPreset extends CommandBase{
    private final ArmSubsystem arm;
    ArmSubsystem.Height height;
    
    

    public SetArmHeightPreset(ArmSubsystem arm, ArmSubsystem.Height height){
        this.arm = arm;
        addRequirements(arm);
        this.height = height;
    }


    @Override
    public void initialize(){
        arm.changeHeight(height);
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
