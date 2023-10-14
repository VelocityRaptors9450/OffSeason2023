package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase{
    private final IntakeSubsystem intake;
    private boolean ramp;
    Timer timer = new Timer();

    //private boolean finish;
    
    

    public IntakeCommand(IntakeSubsystem intake){
        this.intake = intake;
        
        addRequirements(intake);
        timer.reset();
        timer.start();
        
    }



    @Override
    public void initialize(){
        ramp = false;

        // if(firstTime){
        //     finish = false;
        //     firstTime = false;
        // }else{
        //     finish = true;
        // }
       
        
    
    }

    @Override
    public void execute(){


        

        

      

        //For velocity if it works
        if(intake.getTemp() > 60){
            intake.setIntakePower(0);
        }else{
            
            if(ramp && intake.getVelocity() < 200){
                intake.setIntakePower(0.05);
            }else{
                intake.setIntakePower(0.4);
                if(intake.getVelocity() > 300 || timer.get() > 1){
                    ramp = true;
                }
            }
        }   
        
    
    }



        
        
        

        
        

    
    @Override
    public void end(boolean interrupted){
       
        
    }

    @Override
    public boolean isFinished(){
        //if its near goal then turn this to true

        // if(finish){
        //     intake.setIntakePower(0);
        //     firstTime = true;
        // }

        // return finish;   
        return false;
    }
}
