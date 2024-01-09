package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NumericConstants;
import frc.robot.Constants.OperatorConstants;

public class ShooterSubsytem extends SubsystemBase
{
  /* Motor Declarations */
  private static CANSparkMax left = new CANSparkMax( OperatorConstants.leftSpin, MotorType.kBrushless );
  private static CANSparkMax right = new CANSparkMax( OperatorConstants.rightSpin, MotorType.kBrushless );

  
  public ShooterSubsytem() {}


  public void shoot()
  {
    left.setInverted( true );

    left.set( NumericConstants.shooterSpeed );
    right.set( NumericConstants.shooterSpeed );
  }


  @Override
  public void periodic() {}
}
