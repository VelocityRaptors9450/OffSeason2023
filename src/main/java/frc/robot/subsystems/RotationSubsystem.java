package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RotationSubsystem extends SubsystemBase{


    private CANSparkMax motor1 = new CANSparkMax(5,MotorType.kBrushless);
    private double currentPIDTime, previousPIDTime, previousRotationError, p = 0.175, i = 0, d = 0;



    public void setPower(double power){
        motor1.set(power);
    }

    public void goToPosition(double target){
        if(getEncoderTics() < target - 0.5){
            setPower(0.3);
        }else if(getEncoderTics() > target + 0.5){
            setPower(-0.3);
        }else{
            setPower(0);
        }
        
    }

    public void pid(double target, double limiter){
        currentPIDTime = System.currentTimeMillis();


        double rotationError = target - getEncoderTics();
       


        //Integral and derivative aren't doing anything at the moment
        double rotationIntegral = 0.5 * (rotationError + previousRotationError) * (currentPIDTime - previousPIDTime);
        
        double rotationDerivative = (rotationError - previousRotationError)/(currentPIDTime - previousPIDTime);
        

        double rotationPower = ((p * rotationError) + (i * rotationIntegral) + (d * rotationDerivative));
        

        //Limiting max power
        if(Math.abs(rotationPower) > limiter){
            if(rotationPower < 0){
                rotationPower = -1 * limiter;
            }else{
                rotationPower = limiter;
            }
        }


        setPower(rotationPower);

        //Setting previous values
        previousPIDTime = currentPIDTime;
        previousRotationError = rotationError;
        




    }

    public double getEncoderTics(){
        return motor1.getEncoder().getPosition();
    }

    @Override
    public void periodic(){
        
    }

    

    
    
}
