package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSetPowerCommand extends CommandBase{
    private final IntakeSubsystem intake;
    private double power;
    
    
    

    public IntakeSetPowerCommand(IntakeSubsystem intake, double power){
        this.intake = intake;
        addRequirements(intake);
        this.power = power;
    }



    @Override
    public void initialize(){
        intake.setIntakePower(power);
        
        
       
        
    
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
