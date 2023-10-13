package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ArmSetTargetCommand extends CommandBase{
    private final ArmSubsystem arm;
    private double target;
    
    
    

    public ArmSetTargetCommand(ArmSubsystem arm, double target){
        this.arm = arm;
        addRequirements(arm);
        this.target = target;
    }



    @Override
    public void initialize(){
        arm.setArmWristGoal(target);
        
        
       
        
    
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
