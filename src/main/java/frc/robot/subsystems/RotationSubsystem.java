// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RotationSubsystem extends SubsystemBase{

    public enum Height{
        LOW,
        MID,
        HIGH,
        GROUND,
    }


    public CANSparkMax intake = new CANSparkMax(43, MotorType.kBrushless);

    private CANSparkMax leftMotor = new CANSparkMax(44,MotorType.kBrushless);
    private CANSparkMax rightMotor = new CANSparkMax(45, MotorType.kBrushless);
    private Height currentHeight = Height.GROUND;
    

    
    private CANSparkMax wristMotor = new CANSparkMax(46,MotorType.kBrushless);
    private CANSparkMax extensionMotor = new CANSparkMax(47,MotorType.kBrushless);

    //TODO: figure out these values
    private double ticsPerArmRevolution = 144, ticsPerWristRevolution = 172.8, lowTics = (50/360) * ticsPerArmRevolution, midTics = (100/360) * ticsPerArmRevolution, highTics = (135/360) * ticsPerArmRevolution, groundTics = (37.4/360) * ticsPerArmRevolution;
    private PIDController wristPID = new PIDController(0.007,  0,0), downWristPID = new PIDController(0.002,0,0);
    private PIDController pid = new PIDController(0.1, 0, 0), downPID = new PIDController(0.0085, 0, 0);
    
    private int maxRotVel = 1; // maxiumum velocity
    private int maxRotAccel = 2; // maximum acceleration
    private double oldRotVel = 0;
    private double oldTime = Timer.getFPGATimestamp();
    private final ProfiledPIDController rotationPIDController =
    new ProfiledPIDController(0.1,0,0,new TrapezoidProfile.Constraints(maxRotVel, maxRotAccel));
    private SimpleMotorFeedforward rotationFeedforward = new SimpleMotorFeedforward(lowTics, highTics, groundTics);
    private ArmFeedforward armRotationFeedforward = new ArmFeedforward(lowTics, highTics, groundTics);



      // private double startPos = motor6.getEncoder().getPosition();
  public static Timer t = new Timer();
  public static Timer l = new Timer();
  public static Timer g = new Timer();
  Timer quit = new Timer();


  public static enum IntakeStep {
    INITIAL_RAMP_UP,
    INITIAL, // rampUpToggle boolean equivalent
    SLOW_INTAKE,
    IS_OUTPUTTING,
    DONE_OUTPUTTING,
  }

  private IntakeStep intakeStep = IntakeStep.INITIAL;
  
  private static double error = 0.0;
  private static double priorError = 0.0;
  private static double proportion = 0.09;
  private static double derivative = 0.000012;
  private static double pdPower = 0.0;  
  private static double timeChange = 0.02;
  private static double oldpos = 0;
  private static boolean temp = true;
  private static boolean isOutput = false;
  private static boolean initial = true;
  private static boolean rampUPToggle = true;
  public static double velocity = 10;
  private static double time = 0;
  
    

    public RotationSubsystem(){
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
        wristMotor.setIdleMode(IdleMode.kBrake);  
        leftMotor.setInverted(true);  
        rightMotor.setInverted(false);  

        
        //Might need this line
        //intake.setInverted(true);
        intake.setIdleMode(IdleMode.kBrake);
        
        //leftMotor.getEncoder().setVelocityConversionFactor();
        // 144 revolutions of motor to 1 rev of arm
        // 360 / 144 = 2.5 revolutions per degree
        
        leftMotor.getEncoder().setPosition(0);
        rightMotor.getEncoder().setPosition(0);
        
        leftMotor.getEncoder().setPositionConversionFactor(2.5);
        rightMotor.getEncoder().setPositionConversionFactor(2.5);
       


    }

    

    public void setMode(IdleMode mode){
        rightMotor.setIdleMode(mode);
        leftMotor.setIdleMode(mode);
    }


   
    public void setPower(double power){
        leftMotor.set(power);
        rightMotor.set(power);
    }

    public void setLeftPower(double power){
        leftMotor.set(power);
    }

    public void setRightPower(double power){
        rightMotor.set(power);
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


    

    public void armPID(){
        

        double target = convertHeightToTics();

        

        double power = 0;

        if(target - getEncoderTics() > 0){
            power = pid.calculate(getEncoderTics(), target);
        }else{
            power = downPID.calculate(getEncoderTics(), target);
        }

        if(Math.abs(power) > 0.2){
            if(power < 0){
                power = -0.2;
            }else{
                power = 0.2;
            }
        }

        setPower(power);
    }

    public void wristPID(){

        double target = (((90/360) * ticsPerWristRevolution) - (getArmAngleDifference() / (2*Math.PI) * ticsPerWristRevolution));

        

        double power = 0;

        if(target - getWristEncoderTics() > 0){
            power = wristPID.calculate(getWristEncoderTics(), target);
        }else{
            power = downWristPID.calculate(getWristEncoderTics(), target);
        }

        if(Math.abs(power) > 0.3){
            if(power < 0){
                power = -0.3;
            }else{
                power = 0.3;
            }
        }


        // stops PID if gets out of proper bounds
        if (getWristEncoderTics() > Constants.upWristBound || getWristEncoderTics() < Constants.lowWristBound) {
            setPower(0);
        } else {
            setPower(power);
        }
       

    }

    public void currentWristPos() {
        SmartDashboard.putNumber("Wrist Position", getWristEncoderTics());
    }


    public void bothPID(double rotationTarget){
        

        double target = convertHeightToTics();
        double armAngleChange = (rotationTarget - getLeftRotPos()) * ticsPerArmRevolution * (2*Math.PI); // this is the angle arm change
        double wristTargetTics = (convertToRads(90) + getArmAngle()) * ticsPerWristRevolution / (2*Math.PI); //units checks out ;), but the +90???
        //startWrist = 90 + 37.4 + 75 = 127.4

        
        double wristPower = 0;
        double armPower = 0;

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

    
    public void armRotation(double goalPosition) {
        double pidValue = rotationPIDController.calculate((getLeftRotPos()+getRightRotPos())/2, goalPosition);
        double velSetpoint = rotationPIDController.getSetpoint().velocity;
        double accel = (velSetpoint - oldRotVel) / (Timer.getFPGATimestamp() - oldTime); 
        double feedForwardVal = rotationFeedforward.calculate(rotationPIDController.getSetpoint().velocity, accel); //takes velocity, and acceleration
        
        runRotMotor(pidValue + feedForwardVal);
        
        // update vars for determining acceleration later
        oldRotVel = rotationPIDController.getSetpoint().velocity;
        oldTime = Timer.getFPGATimestamp(); 
        
    }

    public void armRotationAngle(double goalAngle) {
        double pidValue = rotationPIDController.calculate((getLeftRotPos()+getRightRotPos())/2, goalAngle);
        double velSetpoint = rotationPIDController.getSetpoint().velocity;
        double accel = (velSetpoint - oldRotVel) / (Timer.getFPGATimestamp() - oldTime); 
        double feedForwardVal = rotationFeedforward.calculate(rotationPIDController.getSetpoint().velocity, accel); //takes velocity, and acceleration
        SmartDashboard.putNumber("PID Value", pidValue);
        SmartDashboard.putNumber("Feed Forward", feedForwardVal);
        SmartDashboard.putNumber("Position error", rotationPIDController.getPositionError());
        
        runRotMotor(pidValue + feedForwardVal);
        
        // update vars for determining acceleration later
        oldRotVel = rotationPIDController.getSetpoint().velocity;
        oldTime = Timer.getFPGATimestamp(); 
        
    }

    public void printVelocity() {
        System.out.println("Left Velocity: " +  leftMotor.getEncoder().getVelocity());
        System.out.println("Right Velocity: " +  rightMotor.getEncoder().getVelocity());

    }
    public void printPosition() {
        SmartDashboard.putNumber("RL Position", getLeftRotPos());
        SmartDashboard.putNumber("RR Position", getRightRotPos());
        SmartDashboard.putNumber("RL Conversion", leftMotor.getEncoder().getPositionConversionFactor());
        SmartDashboard.putNumber("RR Conversion", rightMotor.getEncoder().getPositionConversionFactor());

       

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


    public void resetIntakeVars() {
        temp = true;
        isOutput = false;
        initial = true;
        rampUPToggle = true;
        velocity = 0;
    }
    public double rampUp(double time, double powerTo) {
        return 2 * powerTo / (1 + Math.pow(Math.E, -9*(time))) - 0.4;

    }
    
    public double rampDown(double time, double powerFrom) {
        return -(2 * powerFrom / (1 + Math.pow(Math.E, -9*(time - 1))) - 0.4);

    }



    

    public void wristSetPower(double power){
        wristMotor.set(power);
    }

    public double getWristTarget(){
        return ((convertToRads(90) + getArmAngle()) * ticsPerWristRevolution / (2*Math.PI));
    }

    public double getWristTargetAngle(){
        return getWristAngle() * 2*Math.PI / ticsPerWristRevolution;
    }

    public double getWristEncoderTics(){
        return wristMotor.getEncoder().getPosition();
    }

    public void setWristEncoderTics(double tics){
        wristMotor.getEncoder().setPosition(tics);
    }

    public void initialSetWristEncoder(){
        setWristEncoderTics((202.4/360) * ticsPerWristRevolution);
    }

    public double getWristAngle(){
        return (2*Math.PI * getWristEncoderTics())/ticsPerWristRevolution;
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

   

    @Override
    public void periodic(){

    }

    

    
    
}