package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    CANSparkMax motor1 = new CANSparkMax(0,MotorType.kBrushless);
    CANSparkMax motor2 = new CANSparkMax(0,MotorType.kBrushless);

    private double originalPosition1, currentVelocity1, previousTime1, originalPosition2, currentVelocity2, previousTime2;
    

    public ShooterSubsystem(){
        originalPosition1 = 0;
        previousTime1 = System.currentTimeMillis();
        currentVelocity1 = 0;

        originalPosition2 = 0;
        previousTime2 = previousTime1;
        currentVelocity2 = 0;

    }

    public void setPower(double power){
        if(power < 1) power = -1;
        if(power > 1) power = 1;

        motor1.set(power);
        motor2.set(power);
    }

    public void velPD(int target){

    }

    public void updateMotor1Vel(){

        double currentPosition = motor1.getEncoder().getPosition();
        double currentTime = System.currentTimeMillis();
        currentVelocity1 = (currentPosition - originalPosition1) / (1000 * (currentTime - previousTime1));

        originalPosition1 = currentPosition;
        previousTime1 = currentTime;
    }

    public void updateMotor2Vel(){

        double currentPosition = motor2.getEncoder().getPosition();
        double currentTime = System.currentTimeMillis();
        currentVelocity2 = (currentPosition - originalPosition2) / (1000 * (currentTime - previousTime2));

        originalPosition2 = currentPosition;
        previousTime2 = currentTime;
    }

    public double getVelocity1(){
        return currentVelocity1;
    }

    public double getVelocity2(){
        return currentVelocity2;
    }

    
    
}
