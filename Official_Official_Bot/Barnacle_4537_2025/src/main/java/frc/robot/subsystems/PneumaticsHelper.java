// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsHelper extends SubsystemBase {

private Compressor compressor;

  /** Creates a new PneumaticsHelper. */
  public PneumaticsHelper() {
    compressor = new Compressor(PneumaticsModuleType.REVPH);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pneumatic Pressure (PSI)", compressor.getPressure());
  }
}
