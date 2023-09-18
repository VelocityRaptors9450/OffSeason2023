package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RotationSubsystem;
import frc.robot.subsystems.RotationSubsystem.Height;

public class RotationCommand extends CommandBase{

    
    private final Supplier<Boolean> isAPressed2, isYPressed2,isAReleased2, isYReleased2, isXPressed2, leftBumper, rightBumper, leftBumper2, rightBumper2;
    private final Supplier<Double> rightTrigger, leftTrigger;
    private final XboxController controller;
    private final XboxController controller2;
    private final PS4Controller controllerTest;
    private boolean left, right;
    

    private final RotationSubsystem rotation;
    double target;
    double starting, bottom, low, mid, high;

    public RotationCommand(Supplier<Double> rightTrigger, Supplier<Double> leftTrigger,Supplier<Boolean> leftBumper,Supplier<Boolean> rightBumper,Supplier<Boolean> leftBumper2, Supplier<Boolean> rightBumper2, Supplier<Boolean> isAPressed2, Supplier<Boolean> isYPressed2, Supplier<Boolean> isAReleased2, Supplier<Boolean> isYReleased2, Supplier<Boolean> isXPressed2, XboxController controller, XboxController controller2, PS4Controller controllerTest, RotationSubsystem rotation){
        
        
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
        this.controller = controller;
        this.controller2 = controller2;
        this.isAReleased2 = isAReleased2;
        this.isYReleased2 = isYReleased2;
        this.controllerTest = controllerTest;
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
    
/* 
        boolean aButton = isAPressed.get();
        boolean bButton = isBPressed.get();
        boolean xButton = isXPressed.get();
        boolean yButton = isYPressed.get();
        boolean aButton2 = isAPressed2.get();
        boolean yButton2 = isYPressed2.get();
        boolean xButton2 = isXPressed2.get();
*/ 
        
// ***IMPORTANT*** the only way to get this to work is to have no separate callings
// of each method for the PS4Controller class and XboxController class i.e. any
// println statements must be deleted for it to work

        // This controls linkage pushing in and out
        if(controllerTest.getSquareButtonPressed()){
            rotation.extPow(-0.2);
        }
        if(controllerTest.getSquareButtonReleased()){
            rotation.extPow(0);
        }
        if(controllerTest.getCircleButtonPressed()){
            rotation.extPow(0.2);
        }
        if(controllerTest.getCircleButtonReleased()){
            rotation.extPow(0);
        }


        // This controls the rotation of the linkage up and down
        if(leftTrigger.get() > 0.05){
            //Different states(farther out is slower) (closer in is faster)
            rotation.setPower(-leftTrigger.get()*0.08);
        }else if(rightTrigger.get() > 0.05){
            rotation.setPower(rightTrigger.get()*0.25);
        }else {
            if (rotation.getEncoderTics() > 36) {
                rotation.setPower(0.01);
            } else {
                rotation.setPower(0.02);
            }
        }
        
        // Intake out + in
        // if(controller.getRightBumperPressed()) {
        //     rotation.intakeRight.set(0.2);
        //     rotation.intakeLeft.set(0.2);
        // }
        // if (controller.getRightBumperReleased()) {
        //     rotation.intakeRight.set(0);
        //     rotation.intakeLeft.set(0);
        // }   
        // if(controller.getLeftBumperPressed()) {
        //     rotation.intakeRight.set(-0.5);
        //     rotation.intakeLeft.set(-0.5);
        // }
        // if (controller.getLeftBumperReleased()) {
        //     rotation.intakeRight.set(0);
        //     rotation.intakeLeft.set(0);
        // }
        
        // wrist movement
        if(controllerTest.getTriangleButtonPressed()){
            rotation.wristSetPower(0.15);
        }
        if(controllerTest.getCrossButtonPressed()){
            rotation.wristSetPower(-0.14);
        }
        if (controllerTest.getTriangleButtonReleased() || controllerTest.getCrossButtonReleased()){
            rotation.wristSetPower(0.02);
        }
        


        
/*        
        System.out.println("Right side: " + controllerTest.getL2Axis());
        System.out.println("Left side: " + controllerTest.getR2Axis());
        System.out.print(controllerTest.getRawAxis(PS4Controller.Axis.kR2.value));

        // This controls the rotation of the linkage up and down
        if (controllerTest.getL2Axis() > 0.05) {
            // left trigger
            rotation.setPower(controllerTest.getL2Axis()*0.3);
        } else if (controllerTest.getR2Axis() > 0.05) {
            // right trigger
            rotation.setPower(-controllerTest.getR2Axis()*0.08);
        } else {
            rotation.setPower(0.02);

        }
        
*/
        //System.out.print(controllerTest.getRawAxis(PS4Controller.Axis.kR2.value));


        /* 
        if(controller.getLeftBumperPressed()){
            rotation.extPow(-0.3);
        
        }if(controller.getLeftBumperReleased()){
            rotation.extPow(0);
        }
        if(controller.getRightBumperPressed()){
            rotation.extPow(0.5);
        }
        if(controller.getRightBumperReleased()){
            rotation.extPow(0);
        }
        
        if(xButton){
            rotation.extPow(0);
             rotation.wristSetPower(0);
            //rotation.intake(0);
             rotation.intakeRight.set(0);
             rotation.intakeLeft.set(0);
        }
        */
        
        

     
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
        
/* 

        if(controller2.getRightTriggerAxis() > 0.05){
            rotation.wristSetPower(controller2.getRightTriggerAxis()*0.2);
        }  else if(controller2.getLeftTriggerAxis() > 0.05){
            rotation.wristSetPower(-controller2.getLeftTriggerAxis()*0.2);
        }else{
            rotation.wristSetPower(0.02);
        }

*/
       
        // if(xButton2){
        //     rotation.wristSetPower(0);
        //     rotation.intakeRight.set(0);
        //     rotation.intakeLeft.set(0);
        // }
        /*
        if(rightBumper2.get()){
            rotation.intakeRight.set(0.2);
            rotation.intakeLeft.set(0.2);
        }else if(leftBumper2.get()){
            rotation.intakeRight.set(-0.2);
            rotation.intakeLeft.set(-0.2);
        }else{
            rotation.intakeRight.set(0);
            rotation.intakeLeft.set(0);
        }
*/
        

/*
        if(controller2.getRightBumperPressed()) {
            rotation.intakeRight.set(0.2);
            rotation.intakeLeft.set(0.2);
        }

        if (controller2.getRightBumperReleased()) {
            rotation.intakeRight.set(0);
            rotation.intakeLeft.set(0);
        }
        if(controller2.getLeftBumperPressed()) {
            rotation.intakeRight.set(-0.7);
            rotation.intakeLeft.set(-0.7);
        }

        if (controller2.getLeftBumperReleased()) {
            rotation.intakeRight.set(0);
            rotation.intakeLeft.set(0);
        }

*/
        
        // System.out.println("Arm Target: " + rotation.convertHeightToTics());
        // System.out.println("Arm Tics: " + rotation.getEncoderTics());
        // System.out.println("Arm Angle: " + rotation.getArmAngle());
        // System.out.println();
        // System.out.println("Wrist Target: " + rotation.getWristTarget());
        // System.out.println("Wrist Tics: " + rotation.getWristEncoderTics());
        // System.out.println("Wrist Target Angle: " + rotation.getWristTargetAngle());
        // System.out.println("Wrist Angle: " + rotation.getWristAngle());
        
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