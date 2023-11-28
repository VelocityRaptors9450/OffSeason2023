// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class PneumaticsSubsytem extends SubsystemBase
{
  private static final Compressor compressor =
    new Compressor
    (
      PneumaticsConstants.CompressorModuleID,
      PneumaticsModuleType.REVPH
    );

  public PneumaticsSubsytem() {}

  public void Up()
  {
    compressor.enableAnalog
    (
      PneumaticsConstants.MinPSI,
      PneumaticsConstants.MaxPSI
    );
  }

  public void Down() { compressor.disable(); }

  @Override
  public void periodic() {}
}
