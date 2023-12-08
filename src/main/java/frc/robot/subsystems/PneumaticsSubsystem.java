// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class PneumaticsSubsystem extends SubsystemBase
{
  private static final PneumaticHub pHub =
    new PneumaticHub(
      PneumaticsConstants.PneumaticsHubModuleID
    );
   
  // private static final DoubleSolenoid valve =
  //   new DoubleSolenoid(
  //     PneumaticsConstants.PneumaticsHubModuleID,
  //     PneumaticsModuleType.REVPH,
  //     PneumaticsConstants.SolenoidValveForwardChannel,
  //     PneumaticsConstants.SolenoidValveBackwardsChannel
  //   );

  private static final Solenoid valve =
    new Solenoid(
      PneumaticsConstants.PneumaticsHubModuleID,
      PneumaticsModuleType.REVPH,
      PneumaticsConstants.SolenoidValveChannel
    );

  public PneumaticsSubsystem() {}

  // public void setModeExtend()
  //   { valve.set(Value.kForward); }

  // public void setModeRetract()
  //   { valve.set(Value.kReverse); }
  
  public void enableCompressor()
  {
    valve.set(true);

    pHub.enableCompressorAnalog(
      PneumaticsConstants.MinPSI,
      PneumaticsConstants.MaxPSI
    );
  }

  public void disableCompressor()
  {
    pHub.disableCompressor();

    // valve.set(Value.kOff);
    valve.set(false);
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber(
      "Compressor Pressure",
      pHub.getPressure(PneumaticsConstants.AnalogPressureSensorChannel)
    );
  }
}
