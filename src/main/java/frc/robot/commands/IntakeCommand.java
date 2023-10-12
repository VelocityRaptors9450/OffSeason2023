package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase{
    private final IntakeSubsystem intake;
    private boolean firstTime = true;
    private boolean finish;
    
    

    public IntakeCommand(IntakeSubsystem intake){
        this.intake = intake;
        addRequirements(intake);
    }



    @Override
    public void initialize(){

        if(firstTime){
            finish = false;
            firstTime = false;
        }else{
            finish = true;
        }
       
        
    
    }

    @Override
    public void execute(){


        intake.intake(0.25);

        
    
    }



        
        
        

        
        

    
    @Override
    public void end(boolean interrupted){
       
        
    }

    @Override
    public boolean isFinished(){
        //if its near goal then turn this to true

        if(finish){
            intake.setIntakePower(0);
            firstTime = true;
        }

        return finish;   
    }
}
