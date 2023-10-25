package frc.robot.commands;

import java.util.concurrent.SubmissionPublisher;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ArmWristSetTargetCommand extends CommandBase{
    private final ArmSubsystem arm;
    private double armTarget;
    private double wristTarget;
    
    

    public ArmWristSetTargetCommand(ArmSubsystem arm, double armTarget, double wristTarget){
        this.arm = arm;
        addRequirements(arm);
        this.armTarget = armTarget;
        this.wristTarget = wristTarget;
    }


    @Override
    public void initialize(){
        arm.setArmGoal(armTarget);
        arm.setWristGoal(wristTarget);
        
    
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
