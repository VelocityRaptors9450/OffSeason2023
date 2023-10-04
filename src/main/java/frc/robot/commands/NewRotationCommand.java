package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class NewRotationCommand extends CommandBase{

    
     

    private final ArmSubsystem arm;
    private double wristTarget;
    private double rotTarget;
    

    public NewRotationCommand(ArmSubsystem arm, double wristTarget, double rotTarget){
        this.rotTarget = rotTarget;
        this.wristTarget = wristTarget;
        this.arm = arm;
        addRequirements(arm);
    }



    @Override
    public void initialize(){
        // arm.setRotationGoal(rotTarget);

        // arm.setWristGoal(wristTarget);
    
    }

    @Override
    public void execute(){
        
        

        
        

    }

    @Override
    public void end(boolean interrupted){
       
        
    }

    @Override
    public boolean isFinished(){
        //if its near goal then turn this to true
        return true;   
    }
}