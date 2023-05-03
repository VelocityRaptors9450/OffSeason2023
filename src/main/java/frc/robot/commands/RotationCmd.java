package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RotationSubsystem;
import frc.robot.subsystems.SwerveSubsystemKrish;

public class RotationCmd extends CommandBase{

    
    private final Supplier<Boolean> isAPressed, isBPressed, isXPressed, isYPressed;
    
    

    private final RotationSubsystem rotation;
    double target = -7.0952;

    public RotationCmd(Supplier<Boolean> isAPressed, Supplier<Boolean> isBPressed, Supplier<Boolean> isXPressed, Supplier<Boolean> isYPressed, RotationSubsystem rotation){
        
        this.isAPressed = isAPressed;
        this.isBPressed = isBPressed;
        this.isXPressed = isXPressed;
        this.isYPressed = isYPressed;


        this.rotation = rotation;
        
        addRequirements(rotation);
    }



    @Override
    public void initialize(){
       

    }

    @Override
    public void execute(){
        
        boolean aButton = isAPressed.get();
        boolean bButton = isBPressed.get();
        boolean xButton = isXPressed.get();
        boolean yButton = isYPressed.get();

        if(aButton){
            target = -7.0952;

        }

        if(bButton){
            target = -5.5476;

        }

        if(xButton){
            target = -4.0714;

        }

        if(yButton){
            target = -2.5714;

        }
        
        rotation.pid(target, 0.3);

        System.out.println("Target: " + target);
        System.out.println("Current: " + rotation.getEncoderTics());




    }

    @Override
    public void end(boolean interrupted){
       
        
    }

    @Override
    public boolean isFinished(){
        return false;   
    }
}