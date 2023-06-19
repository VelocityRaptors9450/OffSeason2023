package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RotationSubsystem;
import frc.robot.subsystems.SwerveSubsystemKrish;

public class RotationCmd extends CommandBase{

    
    private final Supplier<Boolean> isAPressed, isBPressed, isXPressed, isYPressed;
    private final Supplier<Double> rightTrigger, leftTrigger;
    
    

    private final RotationSubsystem rotation;
    double target;
    double starting, bottom, low, mid, high;

    public RotationCmd(Supplier<Double> rightTrigger, Supplier<Double> leftTrigger,Supplier<Boolean> isAPressed, Supplier<Boolean> isBPressed, Supplier<Boolean> isXPressed, Supplier<Boolean> isYPressed, RotationSubsystem rotation){
        
        this.isAPressed = isAPressed;
        this.isBPressed = isBPressed;
        this.isXPressed = isXPressed;
        this.isYPressed = isYPressed;
        this.rightTrigger = rightTrigger;
        this.leftTrigger = leftTrigger;


        this.rotation = rotation;

        
        
        addRequirements(rotation);
    }



    @Override
    public void initialize(){
        starting = rotation.getStartPosition();
        bottom = starting + 8.5;
        low = bottom - 0.8809;
        mid = bottom - 3.5208;
        high = bottom - 5.2856;
        rotation.setEncoderTics(0);

        target = bottom;
       

    }

    @Override
    public void execute(){

        // double inPowerAmt = -1 * leftTrigger.get() / 3;
        // double outPowerAmt = 1 *  rightTrigger.get() / 3;
        // double totalPowerAmt = inPowerAmt + outPowerAmt;

        // if(Math.abs(totalPowerAmt) > 0.05){
        //     rotation.setPower(totalPowerAmt);
        // }else{
        //     rotation.setPower(0);
        // }

        
        boolean aButton = isAPressed.get();
        boolean bButton = isBPressed.get();
        boolean xButton = isXPressed.get();
        boolean yButton = isYPressed.get();

        if(aButton){
            target = bottom;

        }

        if(bButton){
            target = low;

        }

        if(xButton){
            target = mid;

        }

        if(yButton){
            target = high;

        }
        
        rotation.pid(target);

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