package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RotationSubsystem;
import frc.robot.subsystems.SwerveSubsystemKrish;
import frc.robot.subsystems.RotationSubsystem.Height;

public class RotationCmd extends CommandBase{

    
    private final Supplier<Boolean> isAPressed, isBPressed, isXPressed, isYPressed;
    //private final Supplier<Double> rightTrigger, leftTrigger;
    
    

    private final RotationSubsystem rotation;
    double target;
    double starting, bottom, low, mid, high;

    public RotationCmd(Supplier<Double> rightTrigger, Supplier<Double> leftTrigger,Supplier<Boolean> isAPressed, Supplier<Boolean> isBPressed, Supplier<Boolean> isXPressed, Supplier<Boolean> isYPressed, RotationSubsystem rotation){
        
        this.isAPressed = isAPressed;
        this.isBPressed = isBPressed;
        this.isXPressed = isXPressed;
        this.isYPressed = isYPressed;
        // this.rightTrigger = rightTrigger;
        // this.leftTrigger = leftTrigger;


        this.rotation = rotation;

        
        
        addRequirements(rotation);
    }



    @Override
    public void initialize(){
        
        rotation.initialSetEncoder();
        rotation.initialSetWristEncoder();

        
       

    }

    @Override
    public void execute(){

        // double inPowerAmt = -1 * leftTrigger.get() / 3;
        // double outPowerAmt = 1 *  rightTrigger.get() / 3;
        // double totalPowerAmt = inPowerAmt + outPowerAmt;

        // if(Math.abs(totalPowerAmt) > 0.05){
        //     if(totalPowerAmt > 0.3){
        //         totalPowerAmt = 0.3;
        //     }else if(totalPowerAmt < -0.3){
        //         totalPowerAmt = -0.3;
        //     }
        //     rotation.setPower(totalPowerAmt);
        // }else{
        //     rotation.setPower(0);
        // }

        
        boolean aButton = isAPressed.get();
        boolean bButton = isBPressed.get();
        boolean xButton = isXPressed.get();
        boolean yButton = isYPressed.get();

        if(aButton){
            //target = bottom;
            rotation.changeHeight(Height.GROUND);

        }

        if(bButton){
            //target = low;
            rotation.changeHeight(Height.LOW);


        }

        if(xButton){
            //target = mid;
            rotation.changeHeight(Height.MID);


        }

        if(yButton){
            //target = high;
            rotation.changeHeight(Height.HIGH);


        }

        System.out.println("Arm Target: " + rotation.convertHeightToTics());
        System.out.println("Arm Tics: " + rotation.getEncoderTics());
        System.out.println("Arm Angle: " + rotation.getArmAngle());
        System.out.println();
        System.out.println("Wrist Target: " + rotation.getWristTarget());
        System.out.println("Wrist Tics: " + rotation.getWristEncoderTics());
        System.out.println("Wrist Target Angle: " + rotation.getWristTargetAngle());
        System.out.println("Wrist Angle: " + rotation.getWristAngle());
        
        //rotation.armPID();
        //rotation.wristPID();
        //rotation.bothPID();

        
        //System.out.println("Current: " + rotation.getEncoderTics());




    }

    @Override
    public void end(boolean interrupted){
       
        
    }

    @Override
    public boolean isFinished(){
        return false;   
    }
}