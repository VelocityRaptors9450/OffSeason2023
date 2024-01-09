package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NumericConstants;
import frc.robot.Constants.OperatorConstants;

public class DriveTrain extends SubsystemBase
{
  /* Motor Declarations */
  private final CANSparkMax frontleft = new CANSparkMax(OperatorConstants.frontleft, MotorType.kBrushless);
  private final CANSparkMax backleft = new CANSparkMax(OperatorConstants.backleft, MotorType.kBrushless);

  private final CANSparkMax frontright = new CANSparkMax(OperatorConstants.frontright, MotorType.kBrushless);
  private final CANSparkMax backright = new CANSparkMax(OperatorConstants.backright, MotorType.kBrushless);


  public DriveTrain() {}

  public void move(double leftY, double rightX)
  {
    // Inverts the back motors so they go in the same direction as the front ones.
    backleft.setInverted(false);
    backright.setInverted(false);

    if (leftY > NumericConstants.deadzone) // If using the LEFT stick, the robot will move forwards and backwards.
    {
      frontleft.set( leftY / 1.5 );
      backleft.set( leftY / 1.5 );

      frontright.set( leftY / 1.5 );
      backright.set( leftY / 1.5 );
    }
    else if (rightX > NumericConstants.deadzone) // If using the RIGHT stick, the robot will turn.
    {
      frontleft.set( rightX / 2 );
      backleft.set( rightX / 2 );

      frontright.set( rightX / -2 );
      backright.set( rightX / -2 );
    }
  }


  @Override
  public void periodic() {}
}
