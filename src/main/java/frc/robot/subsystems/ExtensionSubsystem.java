package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExtensionSubsystem extends SubsystemBase{

    PIDController pid = new PIDController(0.025, 0, 0);
    CANSparkMax motor1 = new CANSparkMax(18,MotorType.kBrushless);
    RelativeEncoder encoder = motor1.getEncoder();

    public ExtensionSubsystem(){
        motor1.setIdleMode(IdleMode.kBrake);
    }



    public void setPower(double power) {
        if (position() > 47) {
            System.out.println("Current Pos: " + position());
            if (power > 0) {
                motor1.set(0);
                
            } else {
                motor1.set(power);
            }
        } else if (position() < 2.5) { // means initial position next time will be farther out (by 2.5)
            if (power < 0) {
                motor1.set(0);
                
            } else {
                motor1.set(power);
            }
        } else {
            motor1.set(power);
        }
        
        
        
    }
    public void manualMov(double power, boolean rightTrigger){
        if(position() > 46){
            motor1.set(0);
        }else if(position() <= 0){
            motor1.set(0);
        }else{
            if(rightTrigger){
                motor1.set(power * 0.8);
            }else{
                motor1.set(-power * 0.8);
            }
        }
    }

    public double position(){
        return encoder.getPosition();
    }

    public void pid(double targetPosition){

        double power = pid.calculate(position(), targetPosition);

        if(Math.abs(power) > 0.8){
            if(power < 0){
                power = -0.8;
            }else{
                power = 0.8;
            }
        }

        setPower(power);
    }

    @Override
    public void periodic(){
        
    }

    

    
    
}
