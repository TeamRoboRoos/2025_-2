// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModuleSubsystem extends SubsystemBase {

  private SparkMax turnMotor;
  private SparkClosedLoopController turnController;
  private PIDController turnPidController;

  private SparkMax driveMotor;
  private SparkClosedLoopController driveController;

  private CANcoder turnAbsEncoder;

  /** Creates a new SwerveModule. */
  public SwerveModuleSubsystem(int turnMotorId, int driveMotorId, int turnAbsEncoderId) {

    turnMotor = new SparkMax(turnMotorId, MotorType.kBrushless);
    turnController = turnMotor.getClosedLoopController();
    turnPidController = new PIDController(SwerveConstants.kp, SwerveConstants.ki, SwerveConstants.kd);
    SmartDashboard.putNumber("p", 0);

    driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
    driveController = driveMotor.getClosedLoopController();

    turnAbsEncoder = new CANcoder(turnAbsEncoderId);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    turnPidController.setP(SmartDashboard.getNumber("p", 0));
  }

  public Rotation2d getTurnAngle() {
    return Rotation2d.fromDegrees(turnAbsEncoder.getPosition().getValue().in(Units.Degree));

  }

  public void setSwerveState(SwerveModuleState state) {
    state.optimize(getTurnAngle());
    goDriveSpeed(state.speedMetersPerSecond);
    turnToAngle(state.angle);
  }

  public void goDriveSpeed(double speedMetersPerSecond) {
    // Sets the intended speed
    driveController.setReference(speedMetersPerSecond, ControlType.kVelocity);
  }

  public void turnToAngle(Rotation2d angle) {
    double set = turnPidController.calculate(getTurnAngle().getRotations(), angle.getRotations());
    double p = turnPidController.getP();
    turnController.setReference(set, ControlType.kDutyCycle);

    SmartDashboard.putNumber("set turn sped", p);
  }

}
