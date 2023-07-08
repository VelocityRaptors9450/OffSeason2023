package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RotationSubsystem extends SubsystemBase{

    public enum Height{
        LOW,
        MID,
        HIGH,
        GROUND,
    }


    
    private CANSparkMax motor1 = new CANSparkMax(5,MotorType.kBrushless);
    private Height currentHeight = Height.GROUND;
    

    //TODO: figure out these values
    private CANSparkMax wristMotor = new CANSparkMax(4,MotorType.kBrushless);
    private double ticsPerArmRevolution = 0, ticsPerWristRevolution = 0, lowTics = (50/360) * ticsPerArmRevolution, midTics = (100/360) * ticsPerArmRevolution, highTics = (135/360) * ticsPerArmRevolution, groundTics = (37.4/360) * ticsPerArmRevolution;
    private PIDController wristPID = new PIDController(0, 0,0), downWristPID = new PIDController(0,0,0);
    
    private PIDController pid = new PIDController(0.1, 0, 0), downPID = new PIDController(0.0085, 0, 0);

    

    public RotationSubsystem(){
        //motor1.setIdleMode(IdleMode.kBrake);
        wristMotor.setIdleMode(IdleMode.kBrake);
        
        
    }

    

    

    public void setMode(IdleMode mode){
        motor1.setIdleMode(mode);
    }



    public void setPower(double power){
        motor1.set(power);
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

        if(Math.abs(power) > 0.3){
            if(power < 0){
                power = -0.3;
            }else{
                power = 0.3;
            }
        }

        setPower(power);
    }

    public void wristPID(){

        double target = (((90/360) * ticsPerWristRevolution) - (getArmAngleDifference() / 360 * ticsPerWristRevolution));

        

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

        setPower(power);

    }


    public void bothPID(){
        

        double target = convertHeightToTics();
        double wristTargetTics = (90 + getArmAngle()) * ticsPerWristRevolution / 360;
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

       

        if(wristTargetTics - getEncoderTics() < 0){
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

    public void wristSetPower(double power){
        wristMotor.set(power);
    }

    public double getWristTarget(){
        return ((90 + getArmAngle()) * ticsPerWristRevolution / 360);
    }

    public double getWristTargetAngle(){
        return getWristAngle() * 360 / ticsPerWristRevolution;
    }

    public double getWristEncoderTics(){
        return wristMotor.getEncoder().getPosition();
    }

    public void setWristEncoderTics(double tics){
        wristMotor.getEncoder().setPosition(tics);
    }

    public void initialSetWristEncoder(){
        setEncoderTics((202.4/360) * ticsPerWristRevolution);
    }

    public double getWristAngle(){
        return (360 * getWristEncoderTics())/ticsPerWristRevolution;
    }

    public double getArmAngle(){
        return (360 * getEncoderTics())/ticsPerArmRevolution;
    }

    public double getArmAngleDifference(){
        return (getArmAngle() - (37.4));
    }

    public double getEncoderTics(){
        return motor1.getEncoder().getPosition();
    }

    public void setEncoderTics(double tics){
        motor1.getEncoder().setPosition(tics);
    }

    public void initialSetEncoder(){
        setEncoderTics(groundTics);
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
