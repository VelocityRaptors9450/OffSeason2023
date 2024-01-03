package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
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

    private double armTarget = 0.53;

    //double p = 0;

    //public CANSparkMax intake = new CANSparkMax(Constants.intakeId, MotorType.kBrushless);

    // private CANSparkMax leftMotor = new CANSparkMax(Constants.rotationLeftId,MotorType.kBrushless);
    private CANSparkMax rightMotor = new CANSparkMax(Constants.rotationRightId, MotorType.kBrushless);
    
    
    private boolean runStuff = true;
    
    private CANSparkMax wristMotor = new CANSparkMax(Constants.wristId,MotorType.kBrushless);
    SparkMaxAbsoluteEncoder wristEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
    
    
    private final ProfiledPIDController rotation = new ProfiledPIDController(6, 0, 0, new Constraints(1, 0.5));//maxVel = 3.5 and maxAccel = 2.5
    private final ArmFeedforward rotationFF = new ArmFeedforward(0, 0.027, 0.00001);

    // wrist i guess
    private final ProfiledPIDController wrist = new ProfiledPIDController(2, 0, 0, new Constraints(1.2, 1.2));
    private final ArmFeedforward wristFF = new ArmFeedforward(0, 0.1, 0.027);

    private PIDController wristPID = new PIDController(0.007,  0,0), downWristPID = new PIDController(0.002,0,0);
    private PIDController pid = new PIDController(0.1, 0, 0), downPID = new PIDController(0.0085, 0, 0);
    
    Timer timer = new Timer();
    private double ticsPerArmRevolution = 144, ticsPerWristRevolution = /*172.8*/ 120, lowTics = (50/360) * ticsPerArmRevolution, midTics = (100/360) * ticsPerArmRevolution, highTics = (135/360) * ticsPerArmRevolution, groundTics = (37.4/360) * ticsPerArmRevolution;
    private boolean intialization = true;

    DutyCycleEncoder absEncoder = new DutyCycleEncoder(0);


    public ArmSubsystem(){
        //leftMotor.setSmartCurrentLimit(40);
        rightMotor.setSmartCurrentLimit(40);
        wristMotor.setSmartCurrentLimit(40);
        rightMotor.setIdleMode(IdleMode.kBrake);
        wristMotor.setIdleMode(IdleMode.kBrake);
        // leftMotor.restoreFactoryDefaults();
       


        // leftMotor.setIdleMode(IdleMode.kCoast);
        
        
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
        // leftMotor.getEncoder().setPosition(0);
        


        //timer.start();
        
        
        //setWristGoal(0);
        // if (intialization) {
        rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 300);   //For follower motors
        rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); //Analog Sensor Voltage + Velocity + position
        rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); //Duty cycler velocity + pos
        //rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);        //Duty Cycle Absolute Encoder Position and Abs angle
        rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); //Duty Cycle Absolute Encoder Velocity + Frequency
        
        wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 300); 
        wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
        wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);

        //wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535); 
        wristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); 
        
        setArmGoal(0.53);
        setWristGoal(0.40); //Need to change


    }

    public double getPosition(){
        return absEncoder.get();
    }

    public double getWristPosition() {
        return wristEncoder.getPosition();
    }

    

    public void initialize(){
        rightMotor.restoreFactoryDefaults();

        
        rightMotor.setIdleMode(IdleMode.kBrake);
        wristMotor.setIdleMode(IdleMode.kBrake);  

        //wristMotor.getEncoder().setPosition(0);

        //wrist.reset(getWristAngle());
        rotation.reset(getPosition());
        //setWristGoal(0);

        //initialSetWristEncoder();
           
        setArmGoal(0.53);
        setWristGoal(0.18); //Need to change

    }
    

   

  





    // private double getLeftPosition(){
    //     return leftMotor.getEncoder().getPosition() * -2.5 * Math.PI / 180;
    // }

    // private double getPosition(){
    //     return (getLeftPosition() + getRightPosition()) / 2;
    // }


    // public void setLeftVoltage(double voltage){
    //     leftMotor.setVoltage(-voltage);
    // }

    public void setArmVoltage(double voltage){
        // leftMotor.setVoltage(-voltage);
        rightMotor.setVoltage(voltage);

    }

    public void downManual(){
        armTarget -= 0.01;

        //wrist.setGoal(2.57 - armTarget);
    }

    public void upManual(){
        armTarget += 0.01;

        //wrist.setGoal(2.57 - armTarget);
    }

    public void setArmWristGoal(double armTarget, double wristTarget){
        //rotation.setGoal(target);
        setArmGoal(armTarget);
        setWristGoal(wristTarget);
        // if (target < 0) {
        //     wrist.setGoal(getWristAngle() - (target-0.1));
        // } else if (target > 0) {
        //     wrist.setGoal(getWristAngle() - (target+0.1));
        // }
        // wrist.setGoal(getWristAngle() - (target - getArmAngle()));
        
        //wrist.setGoal(2.57 - target);
        
        /* THIS BELOW NEED TESTING FOR NEW ARM, not SURE IF IT WILL WORK ORIgiONAL CODE IS THE ONE LINE ABOVE */
        //if(target > 2.57){
        //    wrist.setGoal(target - 2.57);
        //}else if(target < 2.57 && target > 0){
        //    wrist.setGoal(2.57 - target);
        //}
 
    }

    public void setArmGoal(double target) {
        //rotation.setGoal(target);
        armTarget = target;
    }

    public double getGoal(){
        return rotation.getGoal().position;
    }

    public double calculateRotationPID(){
        return rotation.calculate(getPosition(), armTarget);
    }
    public double calculateWristFF() {
        return wristFF.calculate(getWristPosition(), wrist.getSetpoint().velocity);
    }
    public double calculateWristPID() {
        return wrist.calculate(getWristPosition(), wrist.getGoal());
    }
    public void setWristGoal(double target) {
        wrist.setGoal(target);
    }
    public void setWristVoltage(double voltage) {
        wristMotor.setVoltage(voltage);
    }

    public void updateRotationOutput(){
        double ffValue = calculateRotationFF();
        double percentOutput = MathUtil.clamp(calculateRotationPID() + ffValue, -1.0, 1.0);
        double voltage = convertToVolts(percentOutput);

        SmartDashboard.putNumber("Rotation FF", ffValue);
        SmartDashboard.putNumber("Rotation Voltage", voltage);
        
        
        setArmVoltage(voltage);
        
        
    }

    public void updateWristOutput(){
        double percentOutput = MathUtil.clamp(calculateWristPID() + calculateWristFF(), -1.0, 1.0);
        double voltage = convertToVolts(percentOutput);

        SmartDashboard.putNumber("Wrist Voltage", voltage);

        // stops PID if gets out of proper bounds
        // if (getWristAngle() > /*upper wrist bound */ Math.PI / 2 || getWristAngle() < /*lower wrist bound */ -Math.PI / 2) {
        //     setWristVoltage(0);
        // } else {
        //     setWristVoltage(voltage);
        // }
        setWristVoltage(voltage);
    }

    



    

    public double calculateRotationFF(){
        return rotationFF.calculate(/*getPosition()*/getPosition(), rotation.getSetpoint().velocity);

        
    }
    

    private double convertToVolts(double percentOutput){
        return percentOutput * Robot.getInstance().getVoltage();
    }

    @Override
    public void periodic(){
        
        if(runStuff){
            updateRotationOutput();
            updateWristOutput();

        }else{
            setArmVoltage(0);
           
        }
        // SmartDashboard.putNumber("LeftPosition", getLeftPosition());
        SmartDashboard.putNumber("Right Arm Position", getPosition());
        SmartDashboard.putNumber("Target", armTarget);
        //SmartDashboard.putNumber("Wrist Error", wrist.getPositionError());
        //SmartDashboard.putNumber("Position Error", rotation.getPositionError());
        
        //SmartDashboard.putNumber("Wrist Position", getWristPosition());
        //SmartDashboard.putNumber("Arm current", rightMotor.getOutputCurrent());
        

        
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
    // public double getLeftRotPos() {
    //     return leftMotor.getEncoder().getPosition() ;
    // }

    public void wristSetPower(double power){
        wristMotor.set(power);
    }

    public double wristAngletoPosTarget(double angleRads) {
        return angleRads * ticsPerWristRevolution / (2*Math.PI);
    }

    public double convertToRads(double angle) {
        return angle/360*2*Math.PI;
    }

    // --------------------------------------------------------------

    public void changeHeight(Height height){
        currentHeight = height;
    }

    public Height getHeight(){
        return currentHeight;
    }

    public void goToHeight() {
        if(currentHeight == Height.HIGH){
            setArmGoal(0.5); //Need to change
            setWristGoal(0.28);
        }else if(currentHeight == Height.MID){
            setArmGoal(0.5); //Need to change
            setWristGoal(0.45);
        }else if(currentHeight == Height.LOW){
            setArmGoal(0); //Need to change
            setWristGoal(0.67);
        }else{
            setArmGoal(0.285);
            setWristGoal(0.58);
        }
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
        // leftMotor.set(power);
        rightMotor.set(power);
    }

}