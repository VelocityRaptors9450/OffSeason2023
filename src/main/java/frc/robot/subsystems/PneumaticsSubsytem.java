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

public class PneumaticsSubsytem extends SubsystemBase
{
  // private static final Compressor compressor =
  //   new Compressor
  //   ( PneumaticsConstants.CompressorModuleID,
  //     PneumaticsModuleType.REVPH );

  private static final PneumaticHub pHub =
    new PneumaticHub(
      PneumaticsConstants.PneumaticsHubModuleID
    );
    
  private static final Solenoid valve =
    new Solenoid(
      PneumaticsModuleType.REVPH,
      PneumaticsConstants.SolenoidValveChannel
    );

  public PneumaticsSubsytem() {}

  public void CompressorPressureUp()
  {
    valve.set(true);

    // compressor.enableAnalog(
    //   PneumaticsConstants.MinPSI,
    //   PneumaticsConstants.MaxPSI
    // );

    pHub.enableCompressorAnalog(
      PneumaticsConstants.MinPSI,
      PneumaticsConstants.MaxPSI
    );
  }

  public void CompressorPressureDown()
  { 
    valve.set(false);

    // compressor.disable();
    pHub.disableCompressor();
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
