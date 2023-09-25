package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ArmSubsystem extends SubsystemBase{


    //double p = 0;

    public CANSparkMax intake = new CANSparkMax(Constants.intakeId, MotorType.kBrushless);

    private CANSparkMax leftMotor = new CANSparkMax(Constants.rotationLeftId,MotorType.kBrushless);
    private CANSparkMax rightMotor = new CANSparkMax(Constants.rotationRightId, MotorType.kBrushless);
    

    
    private CANSparkMax wristMotor = new CANSparkMax(Constants.wristId,MotorType.kBrushless);
    private CANSparkMax extensionMotor = new CANSparkMax(Constants.extensionId,MotorType.kBrushless);

    private final ProfiledPIDController rotation = new ProfiledPIDController(0, 0, 0, new Constraints(1, 1));
    private final ArmFeedforward rotationFF = new ArmFeedforward(0, 0, 0);
    Timer timer = new Timer();
    

    public ArmSubsystem(){
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();


        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
        wristMotor.setIdleMode(IdleMode.kBrake);  
        
        //leftMotor.setInverted(true);  
        //rightMotor.setInverted(false);  

        extensionMotor.setIdleMode(IdleMode.kBrake);
        
        //Might need this line
        intake.setIdleMode(IdleMode.kBrake);
        
        //leftMotor.getEncoder().setVelocityConversionFactor();
        // 144 revolutions of motor to 1 rev of arm
        // 360 / 144 = 2.5 revolutions per degree
        
        leftMotor.setInverted(false);
        leftMotor.getEncoder().setPosition(0);
        rightMotor.getEncoder().setPosition(0);


        timer.start();
        rotation.reset(getLeftPosition());
        
       


    }

    private double getLeftPosition(){
        return leftMotor.getEncoder().getPosition() * -2.5 * Math.PI / 180;
    }

    public void setLeftVoltage(double voltage){
        leftMotor.setVoltage(-voltage);
    }

    private double getRightPosition(){
        return rightMotor.getEncoder().getPosition() * 2.5 * Math.PI / 180;
    }

    public void setRotationGoal(double target){
        rotation.setGoal(target);
    }

    public double calculateRotationPID(){
        return rotation.calculate(getLeftPosition(), rotation.getGoal());
    }

    public void updateRotationOutput(){
        double percentOutput = MathUtil.clamp(calculateRotationPID() + calculateRotationFF(), 0.0, 1.0);
        double voltage = convertToVolts(percentOutput);
        leftMotor.setVoltage(voltage);
        //rightMotor
    }

    public double calculateRotationFF(){
        return rotationFF.calculate(getLeftPosition(), rotation.getSetpoint().velocity);

        
    }

    private double convertToVolts(double percentOutput){
        return percentOutput * Robot.getInstance().getVoltage();
    }

    @Override
    public void periodic(){
        //updateRotationOutput();
        //SmartDashboard.putNumber("LeftPosition", getLeftPosition());
        
    }
    
    


    // public void example(){
    //     Shuffleboard.getTab("Arm")
    //     .add("Arm P value", p)
    //     .withSize(2, 1)
    //     .withWidget(BuiltInWidgets.kNumberSlider)
    //     .withProperties(Map.of("Min", 0, "Max", 0.025))
    //     .getEntry();
        
    // }
    

}