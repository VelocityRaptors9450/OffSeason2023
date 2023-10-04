package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ArmSubsystem extends SubsystemBase{
    public enum Height{
        LOW,
        MID,
        HIGH,
        GROUND,
    }
    private Height currentHeight = Height.GROUND;


    //double p = 0;

    //public CANSparkMax intake = new CANSparkMax(Constants.intakeId, MotorType.kBrushless);

    private CANSparkMax leftMotor = new CANSparkMax(Constants.rotationLeftId,MotorType.kBrushless);
    private CANSparkMax rightMotor = new CANSparkMax(Constants.rotationRightId, MotorType.kBrushless);
    

    
    private CANSparkMax wristMotor = new CANSparkMax(Constants.wristId,MotorType.kBrushless);
    //private CANSparkMax extensionMotor = new CANSparkMax(Constants.extensionId,MotorType.kBrushless);

    private final ProfiledPIDController rotation = new ProfiledPIDController(1.6, 0, 0, new Constraints(1, 1));
    private final ArmFeedforward rotationFF = new ArmFeedforward(0, 0, 0);

    // wrist i guess
    private final ProfiledPIDController wrist = new ProfiledPIDController(0.6, 0, 0, new Constraints(1, 1));
    private final ArmFeedforward wristFF = new ArmFeedforward(0, 0.054, 0.027);

    private PIDController wristPID = new PIDController(0.007,  0,0), downWristPID = new PIDController(0.002,0,0);
    private PIDController pid = new PIDController(0.1, 0, 0), downPID = new PIDController(0.0085, 0, 0);
    
    Timer timer = new Timer();
    private double ticsPerArmRevolution = 144, ticsPerWristRevolution = /*172.8*/ 120, lowTics = (50/360) * ticsPerArmRevolution, midTics = (100/360) * ticsPerArmRevolution, highTics = (135/360) * ticsPerArmRevolution, groundTics = (37.4/360) * ticsPerArmRevolution;
    private boolean intialization = true;

    public ArmSubsystem(){
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();


        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kCoast);
        wristMotor.setIdleMode(IdleMode.kBrake);  
        
        //leftMotor.setInverted(true);  
        //rightMotor.setInverted(false);  

        //extensionMotor.setIdleMode(IdleMode.kBrake);
        
        //Might need this line
        //intake.setIdleMode(IdleMode.kBrake);
        
        //leftMotor.getEncoder().setVelocityConversionFactor();
        // 144 revolutions of motor to 1 rev of arm
        // 360 / 144 = 2.5 degrees / arm revolution
        // 360 / 120 = 3 degrees / wrist revolution
        
        //leftMotor.setInverted(false);
        leftMotor.getEncoder().setPosition(0);
        rightMotor.getEncoder().setPosition(0);


        //timer.start();
        rotation.reset(getLeftPosition());
        wrist.reset(getWristAngle());

        setArmWristGoal(0);
        setWristGoal(0);
        // if (intialization) {
        initialSetWristEncoder();
            // intialization = false;
        // }
       


    }





    private double getLeftPosition(){
        return leftMotor.getEncoder().getPosition() * -2.5 * Math.PI / 180;
    }

    public void setLeftVoltage(double voltage){
        leftMotor.setVoltage(-voltage);
    }

    private double getRightPosition(){
        return rightMotor.getEncoder().getPosition() * 2.5 * Math.PI / 180;
    }

    public void setArmWristGoal(double target){
        rotation.setGoal(target);
        if (target < 0) {
            wrist.setGoal(getWristAngle() - (target-0.1));
        } else if (target > 0) {
            wrist.setGoal(getWristAngle() - (target+0.1));
        }
        
    }

    public void setArmGoal(double target) {
        rotation.setGoal(target);
    }

    public double calculateRotationPID(){
        return rotation.calculate(getLeftPosition(), rotation.getGoal());
    }
    public double calculateWristFF() {
        return wristFF.calculate(getWristAngle(), wrist.getSetpoint().velocity);
    }
    public double calculateWristPID() {
        return wrist.calculate(getWristAngle(), wrist.getGoal());
    }
    public void setWristGoal(double target) {
        wrist.setGoal(target);
    }
    public void setWristVoltage(double voltage) {
        wristMotor.setVoltage(voltage);
    }

    public void updateRotationOutput(){
        double percentOutput = MathUtil.clamp(calculateRotationPID() + calculateRotationFF(), -1.0, 1.0);
        double voltage = -convertToVolts(percentOutput);

        SmartDashboard.putNumber("Rotation Voltage", voltage);
        leftMotor.setVoltage(voltage);
        //rightMotor
    }

    public void updateWristOutput(){
        double percentOutput = MathUtil.clamp(calculateWristPID() + calculateWristFF(), -1.0, 1.0);
        double voltage = convertToVolts(percentOutput);

        SmartDashboard.putNumber("Wrist Voltage", voltage);

        // stops PID if gets out of proper bounds
        if (getWristAngle() > /*upper wrist bound */ Math.PI / 2 || getWristAngle() < /*lower wrist bound */ -Math.PI / 2) {
            setWristVoltage(0);
        } else {
            setWristVoltage(voltage);
        }
    }

    
    

    public void wristPID(double armAngleRads){

        double target = (getWristAngle()) - wristAngletoPosTarget(armAngleRads); ; 
        

        double power = 0;

        if(target - getWristAngle() > 0){
            power = wristPID.calculate(getWristAngle(), target);
        }else{
            power = downWristPID.calculate(getWristAngle(), target);
        }

        if(Math.abs(power) > 0.3){
            if(power < 0){
                power = -0.3;
            }else{
                power = 0.3;
            }
        }


        // stops PID if gets out of proper bounds
        if (getWristAngle() > /*upper wrist bound */ (wristAngletoPosTarget(Math.PI / 2)-2) || getWristAngle() < /*lower wrist bound */ (wristAngletoPosTarget(-Math.PI / 2)+2)) {
            setPower(0);
        } else {
            setPower(power);
        }
       

    }



    public void bothPID(double rotationTarget){
        

        double target = convertHeightToTics();
        double armAngleChange = (rotationTarget - getLeftRotPos()) / ticsPerArmRevolution * (2*Math.PI); // this is the angle arm change
        double wristTargetTics = (getWristAngle()) - wristAngletoPosTarget(armAngleChange); 
        //startWrist = 90 + 37.4 + 75 = 127.4

        
        double wristPower = 0;
        double armPower = 0;
        // arm section
        if(target - getEncoderTics() > 0){
            armPower = pid.calculate(getEncoderTics(), target);
        }else{
            armPower = downPID.calculate(getEncoderTics(), target);
        }

        if(Math.abs(armPower) > 0.3){
            if(armPower < 0){
                armPower = -0.3;
            }else{
                armPower = 0.3;
            }
        }

       
        // wrist section
        if(wristTargetTics - getEncoderTics() > 0){
            wristPower = wristPID.calculate(getEncoderTics(), wristTargetTics);
        }else{
            wristPower = downWristPID.calculate(getEncoderTics(), wristTargetTics);
        }


        if(Math.abs(wristPower) > 0.3){
            if(wristPower < 0){
                wristPower = -0.3;
            }else{
                wristPower = 0.3;
            }
        }

        //setPower(armPower);
        //wristSetPower(wristPower);
    }

    public double calculateRotationFF(){
        return rotationFF.calculate(getLeftPosition(), rotation.getSetpoint().velocity);

        
    }
    

    private double convertToVolts(double percentOutput){
        return percentOutput * Robot.getInstance().getVoltage();
    }

    @Override
    public void periodic(){
        updateRotationOutput();
        updateWristOutput();
        SmartDashboard.putNumber("LeftPosition", getLeftPosition());
        SmartDashboard.putNumber("Target?", rotation.getGoal().position);;
        SmartDashboard.putNumber("Position Error", rotation.getPositionError());
        
        SmartDashboard.putNumber("Wrist Position", getWristAngle());
        

        
    }
    
    


    // public void example(){
    //     Shuffleboard.getTab("Arm")
    //     .add("Arm P value", p)
    //     .withSize(2, 1)
    //     .withWidget(BuiltInWidgets.kNumberSlider)
    //     .withProperties(Map.of("Min", 0, "Max", 0.025))
    //     .getEntry();
        
    // }
    












    public void anEmptyMethod() {
        // for testing
    }
    public double getLeftRotPos() {
        return leftMotor.getEncoder().getPosition() ;
    }

    public double getRightRotPos() {
        return rightMotor.getEncoder().getPosition();
    }

    public void runRotMotor(double voltage) {
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(-voltage);
    }

    public void wristSetPower(double power){
        wristMotor.set(power);
    }

    public double getWristTarget(){
        return ((convertToRads(90) + getArmAngle()) * ticsPerWristRevolution / (2*Math.PI));
    }

    public double wristAngletoPosTarget(double angleRads) {
        return angleRads * ticsPerWristRevolution / (2*Math.PI);
    }


    public double getWristAngle(){
        return wristMotor.getEncoder().getPosition() * 3 * Math.PI / 180; // returns radians
    }

    public void setWristEncoderTics(double tics){
        wristMotor.getEncoder().setPosition(tics);
    }

    public void initialSetWristEncoder(){
        setWristEncoderTics((-90/360) * ticsPerWristRevolution); // intial position -90 degrees
    }

    
    public double getArmAngle(){
        return (2*Math.PI * getEncoderTics())/ticsPerArmRevolution;
    }

    public double getArmAngleDifference(){
        return (getArmAngle() - (convertToRads(37.4)));
    }
    public double convertToRads(double angle) {
        return angle/360*2*Math.PI;
    }

    public double getEncoderTics(){
      return (leftMotor.getEncoder().getPosition() + rightMotor.getEncoder().getPosition()) / 2;
    }

    // --------------------------------------------------------------
    public void setEncoderTics(double tics){
        rightMotor.getEncoder().setPosition(tics);
        leftMotor.getEncoder().setPosition(tics);
    }

    public void initialSetEncoder(){
        //setEncoderTics(groundTics);
    }

    public void changeHeight(Height height){
        currentHeight = height;
    }

    public Height getHeight(){
        return currentHeight;
    }

    public double convertHeightToTics(){
        if(currentHeight == Height.HIGH){
            return highTics; //37.071

        }else if(currentHeight == Height.MID){
            return midTics; //27.118

        }else if(currentHeight == Height.LOW){
            return lowTics; //14.452

        }else{
            return groundTics; //0

        }
    }


    public void setPower(double power){
        leftMotor.set(power);
        rightMotor.set(power);
    }

}