// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class PneumaticsSubsystem extends SubsystemBase
{
  private static final PneumaticHub pHub =
    new PneumaticHub(
      PneumaticsConstants.PneumaticsHubModuleID
    );

  private static final Solenoid forward =
    new Solenoid(
      PneumaticsConstants.PneumaticsHubModuleID,
      PneumaticsModuleType.REVPH,
      PneumaticsConstants.SolenoidValveForwardChannel
    );

  private static final Solenoid backward =
    new Solenoid(
      PneumaticsConstants.PneumaticsHubModuleID,
      PneumaticsModuleType.REVPH,
      PneumaticsConstants.SolenoidValveBackwardsChannel
    );

  public PneumaticsSubsystem() {}
  
  public void disableCompressor()
  {
    pHub.disableCompressor();

    forward.set(false);
    backward.set(false);
  }

  public void extend()
  {
    forward.set(true);
    backward.set(false);

    pHub.enableCompressorAnalog
    (
      PneumaticsConstants.MinPSI,
      PneumaticsConstants.MaxPSI
    );
  }

  public void retract()
  {
    forward.set(false);
    backward.set(false);

    pHub.enableCompressorAnalog
    (
      PneumaticsConstants.MinPSI,
      PneumaticsConstants.MaxPSI
    );
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
