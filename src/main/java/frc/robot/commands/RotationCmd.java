package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RotationSubsystem;
import frc.robot.subsystems.SwerveSubsystemKrish;
import frc.robot.subsystems.RotationSubsystem.Height;

public class RotationCmd extends CommandBase{

    
    private final Supplier<Boolean> isAPressed, isBPressed, isXPressed, isYPressed,isAPressed2, isYPressed2, isXPressed2, leftBumper, rightBumper, leftBumper2, rightBumper2;
    private final Supplier<Double> rightTrigger, leftTrigger;
    
    private boolean left, right;
    

    private final RotationSubsystem rotation;
    double target;
    double starting, bottom, low, mid, high;

    public RotationCmd(Supplier<Double> rightTrigger, Supplier<Double> leftTrigger,Supplier<Boolean> isAPressed, Supplier<Boolean> isBPressed, Supplier<Boolean> isXPressed, Supplier<Boolean> isYPressed,Supplier<Boolean> leftBumper,Supplier<Boolean> rightBumper,Supplier<Boolean> leftBumper2, Supplier<Boolean> rightBumper2, Supplier<Boolean> isAPressed2, Supplier<Boolean> isYPressed2, Supplier<Boolean> isXPressed2,RotationSubsystem rotation){
        
        this.isAPressed = isAPressed;
        this.isBPressed = isBPressed;
        this.isXPressed = isXPressed;
        this.isYPressed = isYPressed;
        this.rightTrigger = rightTrigger;
        this.leftTrigger = leftTrigger;
        this.leftBumper = leftBumper;
        this.rightBumper = rightBumper;
        this.leftBumper2 = leftBumper2;
        this.rightBumper2 = rightBumper2;
        this.rotation = rotation;
        this.isAPressed2 = isAPressed2;
        this.isYPressed2 = isYPressed2;
        this.isXPressed2 = isXPressed2;
        
        
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
        boolean aButton2 = isAPressed2.get();
        boolean yButton2 = isYPressed2.get();
        boolean xButton2 = isXPressed2.get();
        if(aButton){
            rotation.extPow(0.2);
        }
        if(bButton){
            rotation.extPow(-0.2);
        }
        if(xButton){
            rotation.extPow(0);
            rotation.wristSetPower(0);
            //rotation.intake(0);
            rotation.intakeRight.set(0);
            rotation.intakeLeft.set(0);
        }
        if(yButton){
            //rotation.intake(0.4);
            


        }
        /* 
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
        */
        if(leftTrigger.get() > 0.05){
            rotation.setPower(-leftTrigger.get()*0.05);
        }else if(rightTrigger.get() > 0.05){
            rotation.setPower(rightTrigger.get()*0.2);
        }else{
            rotation.setPower(0.02);
        }

     

        if(aButton2){
            rotation.wristSetPower(-0.2);
        }
        
        if(yButton2){
            rotation.wristSetPower(0.2);
        }
        if(xButton2){
            rotation.wristSetPower(0);
        }

        if(rightBumper2.get()){
            rotation.intakeRight.set(0.2);
            rotation.intakeLeft.set(0.2);
        }
        if(leftBumper2.get()){
            rotation.intakeRight.set(-0.2);
            rotation.intakeLeft.set(-0.2);
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