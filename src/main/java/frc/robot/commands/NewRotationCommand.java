package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.RotationSubsystem;
import frc.robot.subsystems.RotationSubsystem.Height;

public class NewRotationCommand extends CommandBase{

    
     

    private final ArmSubsystem arm;
    private double target;
    

    public NewRotationCommand(ArmSubsystem arm, double target){
        this.target = target;
        this.arm = arm;
        addRequirements(arm);
    }



    @Override
    public void initialize(){
        arm.setRotationGoal(target);
    
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