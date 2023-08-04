package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExtensionSubsystem extends SubsystemBase{

    PIDController pid = new PIDController(0.025, 0, 0);
    CANSparkMax motor1 = new CANSparkMax(19,MotorType.kBrushless);
    RelativeEncoder encoder = motor1.getEncoder();

    public ExtensionSubsystem(){
        motor1.setIdleMode(IdleMode.kBrake);
    }



    public void setPower(double power){
        motor1.set(power);
    }

    public double position(){
        return encoder.getPosition();
    }

    public void pid(double targetPosition){

        double power = pid.calculate(position(), targetPosition);

        if(Math.abs(power) > 0.3){
            if(power < 0){
                power = -0.3;
            }else{
                power = 0.3;
            }
        }

        setPower(power);
    }

    @Override
    public void periodic(){
        
    }

    

    
    
}
