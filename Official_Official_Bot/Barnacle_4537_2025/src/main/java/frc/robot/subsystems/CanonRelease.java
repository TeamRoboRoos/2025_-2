// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CanonRelease extends SubsystemBase {

  private Servo deployerServo;

  /** Creates a new AlgaeRemover. */
  public CanonRelease() {
    deployerServo = new Servo(1);
    deployerServo.set(0.0); // TODO Check servo positions
  }

  public Command deployAlgaeRemover() {
    return runOnce(() -> deployerServo.set(0.6));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
