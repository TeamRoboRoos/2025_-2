// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeRemover extends SubsystemBase {

  private Servo deployerServo;
  private SparkMax algaeRemoverMotor;

  /** Creates a new AlgaeRemover. */
  public AlgaeRemover() {
    deployerServo = new Servo(0);
    deployerServo.set(0.0); // TODO Check servo positions

    //algaeRemoverMotor = new SparkMax(19, MotorType.kBrushed); // TODO spark max ID
  }

  public Command deployAlgaeRemover() {
    return runOnce(() -> deployerServo.set(0.8));
  }

  public Command runAlgaeRemoverMotor() {
    return runEnd(() -> algaeRemoverMotor.set(0.5), () -> algaeRemoverMotor.set(0.0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
