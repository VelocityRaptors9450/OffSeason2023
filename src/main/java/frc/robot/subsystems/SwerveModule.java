package frc.robot.subsystems;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ModuleConversion;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

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

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        turnMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setIdleMode(IdleMode.kBrake);

        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turningMotorReversed);
        
        driveEncoder =  driveMotor.getEncoder();
        turnEncoder =  turnMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConversion.DriveEncoderRot2Meter);
        driveEncoder.setPositionConversionFactor(ModuleConversion.DriveEncoderRPM2MeterPerSec);
        turnEncoder.setPositionConversionFactor(ModuleConversion.TurnEncoderRot2Rad);
        turnEncoder.setPositionConversionFactor(ModuleConversion.TurnEncoderRPM2RadPerSec);

        turnPIDController = new PIDController(0.3,0,0);        

        


    }

}
