package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{


    CANSparkMax motor1 = new CANSparkMax(13,MotorType.kBrushless);
    CANSparkMax motor2 = new CANSparkMax(7,MotorType.kBrushless);


    CANSparkMax linkage;
    // = new CANSparkMax(4,MotorType.kBrushless);

    

    RelativeEncoder linkageEncoder;
    // = linkage.getEncoder();


    //Velocity Constants
    private double originalPosition1, currentVelocity1, previousTime1, originalPosition2, currentVelocity2, previousTime2;

    //PID Update Constants
    private double pVel, dVel, previousPIDTime, currentPIDTime, previousPIDError1, previousPIDError2;
    

    public ShooterSubsystem(){
        originalPosition1 = 0;
        previousTime1 = System.currentTimeMillis();
        currentVelocity1 = 0;

        originalPosition2 = 0;
        previousTime2 = previousTime1;
        currentVelocity2 = 0;

        //linkage.setIdleMode(IdleMode.kCoast);
        //linkageEncoder.setPosition(0);

        

        

    }

    public void setLinkagePower(double power){
        linkage.set(power);
    }

    public double getLinkagePosition(){
        return linkageEncoder.getPosition();
    }

    public void setPower(double power){
        if(power < -1) power = -1;
        if(power > 1) power = 1;

        motor1.set(-power);
        motor2.set(power);
    }

    public void setPower(double power1, double power2){
        if(power1 < 1) power1 = -1;
        if(power1 > 1) power1 = 1;

        if(power1 < 1) power2 = -1;
        if(power2 > 1) power2 = 1;

        motor1.set(-power1);
        motor2.set(power2);
    }

    

    public double getPower(){
        return (-motor1.get() + motor2.get()) / 2;
        
    }

    public double getMotor1Power(){
        return motor1.get();
    }

    public double getMotor2Power(){
        return motor2.get();
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

    public void velPDUpdate(double targetVel, double limiter){
        currentPIDTime = System.currentTimeMillis();

        double error1 = targetVel - getVelocity1();
        double error2 = targetVel - getVelocity2();

        double derivative1 = (error1 - previousPIDError1)/(currentPIDTime - previousPIDTime);
        double derivative2 = (error2 - previousPIDError2)/(currentPIDTime - previousPIDTime);

        double power1 = ((pVel * error1) + (dVel * derivative1));
        double power2 = ((pVel * error2) + (dVel * derivative2));

        if(Math.abs(power1) > limiter){
            if(power1 < 0){
                power1 = -1 * limiter;
            }else{
                power1 = limiter;
            }
        }

        if(Math.abs(power2) > limiter){
            if(power2 < 0){
                power2 = -1 * limiter;
            }else{
                power2 = limiter;
            }
        }

        setPower(power1, power2);

        previousPIDTime = currentPIDTime;
        previousPIDError1 = error1;
        previousPIDError2 = error2;



        
    }


    @Override
    public void periodic(){
        
    }

    

    
    
}
