package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExtensionSubsystem extends SubsystemBase{


    CANSparkMax motor1 = new CANSparkMax(18,MotorType.kBrushless);



    public void setPower(double power){
        motor1.set(power);
    }

    @Override
    public void periodic(){
        
    }

    

    
    
}
