package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeStopAndResetCommand extends CommandBase{
    private final IntakeSubsystem intake;
    
    
    

    public IntakeStopAndResetCommand(IntakeSubsystem intake){
        this.intake = intake;
        addRequirements(intake);
    }



    @Override
    public void initialize(){
        intake.setIntakePower(0);
        intake.resetIntakeVars();
        
       
        
    
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
