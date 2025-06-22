// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class test extends SubsystemBase {
  /** Creates a new test. */
  public DoubleSolenoid solly;
  public test() {
    solly = new DoubleSolenoid(0, PneumaticsModuleType.REVPH, 0, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    solly.set(Value.kForward);
  }
}
