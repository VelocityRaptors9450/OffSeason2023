// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class PneumaticsSubsytem extends SubsystemBase
{
  // private static final Compressor compressor =
  //   new Compressor
  //   ( PneumaticsConstants.CompressorModuleID,
  //     PneumaticsModuleType.REVPH );

  private static final PneumaticHub pHub = new PneumaticHub();

  public PneumaticsSubsytem() {}

  public void Up()
  {
    // compressor.enableHybrid(
    //   PneumaticsConstants.MinPSI,
    //   PneumaticsConstants.MaxPSI
    // );

    pHub.enableCompressorAnalog(
      PneumaticsConstants.MinPSI,
      PneumaticsConstants.MaxPSI
    );
  }

  public void Down()
  { 
    // compressor.disable();
    pHub.disableCompressor();
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("Compressor Pressure", pHub.getPressure(0));
  }
}
