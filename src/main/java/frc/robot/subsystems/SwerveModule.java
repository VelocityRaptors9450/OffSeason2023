package frc.robot.subsystems;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.ConversionConstants;
import frc.robot.DriveConstants;


public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;
    private final CANCoder absEncoder;

    private final PIDController turnPIDController;
    
    //connected to analog imports on the roborio(can aceess useing analog import class incode)
   // private final AnalogInput absEncoder;
    private final boolean absEncoderReversed;
    //record how off the wheel is from the forward direction
    private final double absEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed){
        this.absEncoderOffsetRad = absoluteEncoderOffset;
        this.absEncoderReversed = absoluteEncoderReversed;
        //absEncoder = new AnalogInput(absoluteEncoderId);
        absEncoder = new CANCoder(absoluteEncoderId);
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turningMotorReversed);
        
        driveEncoder =  driveMotor.getEncoder();
        turnEncoder =  turnMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ConversionConstants.DriveEncoderRot2Meter);
        driveEncoder.setPositionConversionFactor(ConversionConstants.DriveEncoderRPM2MeterPerSec);
        turnEncoder.setPositionConversionFactor(ConversionConstants.TurnEncoderRot2Rad);
        turnEncoder.setPositionConversionFactor(ConversionConstants.TurnEncoderRPM2RadPerSec);

        /* these values need to be updated*/turnPIDController = new PIDController(0.3,0,0);        
        turnPIDController.enableContinuousInput(-Math.PI, Math.PI);
        

        resetEncoders();
    }
    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }
    public double getTurningPosition(){
        return turnEncoder.getPosition();
    }
    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }
    public double getTurningVelocity(){
        return turnEncoder.getVelocity();
    }
    // THIS ONE IS CONFUZZZLING
    public double getAbsoluteEncoderRad(){
        double angle = absEncoder.getBusVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absEncoderOffsetRad;
        return angle * (absEncoderReversed ? -1.0 : 1.0);
    }
    public void resetEncoders(){
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsoluteEncoderRad());
    }
    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }   
    public void setDesiredState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.PhysicalMaxMetersPerSecond);
        turnMotor.set(turnPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
    }
    public void stop(){
        driveMotor.set(0);
        turnMotor.set(0);
    }
}
