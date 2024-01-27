package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NumericConstants;
import frc.robot.Constants.OperatorConstants;

public class DriveTrain extends SubsystemBase
{
  /* Motor Declarations */
  private final CANSparkMax frontleft = new CANSparkMax( OperatorConstants.frontLeft, MotorType.kBrushless); // Front Left Motor
  private final CANSparkMax frontright = new CANSparkMax( OperatorConstants.frontRight, MotorType.kBrushless); // Front Right Motor

  private final CANSparkMax backleft = new CANSparkMax( OperatorConstants.backLeft, MotorType.kBrushless ); // Back Left Motor
  private final CANSparkMax backright = new CANSparkMax( OperatorConstants.backRight, MotorType.kBrushless ); // Back Right Motor
  
  public final CANSparkMax x = new CANSparkMax( 2, MotorType.kBrushless ); // Back Right Motor



  public DriveTrain() {}

  
  public void move( double leftY, double rightX )
  {
    // Inverts the back motors so they go in the same direction as the front ones.
    backleft.setInverted( false );
    backright.setInverted( false );

    if (leftY > NumericConstants.deadzone) // If using the LEFT stick, the robot will move forwards and backwards.
    {
      frontleft.set( leftY * NumericConstants.driveSpeedLimit );
      backleft.set( leftY * NumericConstants.driveSpeedLimit );

      frontright.set( leftY * NumericConstants.driveSpeedLimit );
      backright.set( leftY * NumericConstants.driveSpeedLimit );
    }
    else if (rightX > NumericConstants.deadzone) // If using the RIGHT stick, the robot will turn.
    {
      frontleft.set( rightX * NumericConstants.turnSpeedLimit );
      backleft.set( rightX * NumericConstants.turnSpeedLimit );

      frontright.set( rightX * NumericConstants.turnSpeedLimit * -1 );
      backright.set( rightX * NumericConstants.turnSpeedLimit * -1 );
    }
  }
  


  @Override
  public void periodic() {}
}
