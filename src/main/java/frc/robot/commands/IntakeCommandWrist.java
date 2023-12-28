package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommandWrist extends CommandBase{
    private final IntakeSubsystem intake;
    private final ArmSubsystem arm;
    private boolean ramp;
    private boolean hasHadCube;
    Timer timer = new Timer();

    //private boolean finish;
    
    

    public IntakeCommandWrist(IntakeSubsystem intake, ArmSubsystem arm){
        this.intake = intake;
        this.arm = arm;
        addRequirements(intake);
        addRequirements(arm);
        timer.reset();
        timer.start();
        
    }



    @Override
    public void initialize(){
        ramp = false;
        hasHadCube = false;
        timer.reset();
        timer.start();
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
                SmartDashboard.putBoolean("Has Cube", true);
                if (!hasHadCube && timer.get() > 2) {
                    arm.setWristGoal(0.65);
                    hasHadCube = true;
                }
            }else{
                intake.setIntakePower(0.5);
                if(intake.getVelocity() > 300 || timer.get() > 1){
                    ramp = true;
                }
                SmartDashboard.putBoolean("Has Cube", false);
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
