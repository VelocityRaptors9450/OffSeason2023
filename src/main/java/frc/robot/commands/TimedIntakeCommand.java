package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class TimedIntakeCommand extends CommandBase{
    private final IntakeSubsystem intake;
    private double power;
    Timer timer = new Timer();
    boolean stop = false;
    
    
    

    public TimedIntakeCommand(IntakeSubsystem intake, double power){
        this.intake = intake;
        addRequirements(intake);
        this.power = power;
    }



    @Override
    public void initialize(){
        stop = false;
        
        intake.setIntakePower(power);
        timer.reset();
        timer.start();
        
        
       
        
    
    }

    @Override
    public void execute(){

        if(timer.get() > 1){
            stop = true;
        }
        

        
    
    }



        
        
        

        
        

    
    @Override
    public void end(boolean interrupted){
       
        
    }

    @Override
    public boolean isFinished(){
        if(stop){
            intake.setIntakePower(0);
        }
        return stop;   
    }
}
