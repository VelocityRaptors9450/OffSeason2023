package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ArmManualCommand extends CommandBase{
    private final ArmSubsystem arm;
    private boolean goUp;
    
    
    

    public ArmManualCommand(ArmSubsystem arm, boolean goUp){
        this.arm = arm;
        addRequirements(arm);
        this.goUp = goUp;
    }



    @Override
    public void initialize(){
        if(goUp){
            arm.upManual();
        }else{
            arm.downManual();
        }
        
        
       
        
    
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
