package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RotationSubsystem extends SubsystemBase{


    private CANSparkMax motor1 = new CANSparkMax(5,MotorType.kBrushless);
    private Timer time;
    private PIDController pid = new PIDController(0.03, 0, 0);
    private double currentPIDTime, previousPIDTime, previousRotationError, rotationIntegral = 0, p = 0.13, i = 0, d = 0.0015, f = 0, startPosition;

    public RotationSubsystem(){
        motor1.setIdleMode(IdleMode.kBrake);
        startPosition = 0;
        time = new Timer();
        time.start();
        time.reset();
        
    }

    public double getStartPosition(){
        return startPosition;
    }



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

    public void pid(double target){
        double power = pid.calculate(getEncoderTics(), target);

        if(Math.abs(power) > 0.3){
            if(power < 0){
                power = -0.3;
            }else{
                power = 0.3;
            }
        }

        setPower(power);
    }

    public void pid(double target, double limiter){
        currentPIDTime = time.get();




        double rotationError = target - getEncoderTics();
       


        //Integral and derivative aren't doing anything at the moment
        rotationIntegral += (0.5 * (rotationError + previousRotationError) * (currentPIDTime - previousPIDTime));


        if(rotationIntegral > 15){
            rotationIntegral = 15;
        }

        if(rotationIntegral < -15){
            rotationIntegral = -15;
        }
        
        double rotationDerivative = (rotationError - previousRotationError)/(currentPIDTime - previousPIDTime);
        
       
        double rotationPower = ((p * rotationError) + (i * rotationIntegral) + (d * rotationDerivative) + (f * (1/(startPosition - getEncoderTics()))));
        System.out.println("Power: " + rotationPower);


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

    public void setEncoderTics(double tics){
        motor1.getEncoder().setPosition(tics);
    }

    @Override
    public void periodic(){
        
    }

    

    
    
}
