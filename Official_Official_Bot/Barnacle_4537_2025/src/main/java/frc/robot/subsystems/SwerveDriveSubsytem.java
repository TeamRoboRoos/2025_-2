// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsytem extends SubsystemBase {

  private SwerveModuleSubsystem frontLeftModule, frontRightModule, backLeftModule, backRightModule;

  /** Creates a new SwerveDriveSubsytem. */
  public SwerveDriveSubsytem() {

    frontLeftModule = new SwerveModuleSubsystem(19, 17, 32);
    // frontRightModule = new SwerveModuleSubsystem(11, 10, 30);
    // backRightModule = new SwerveModuleSubsystem(14, 12, 31);
    // backLeftModule = new SwerveModuleSubsystem(16, 15, 33);

    SmartDashboard.putNumber("driveSpeed", 0);
    SmartDashboard.putNumber("angle", 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double sped = SmartDashboard.getNumber("driveSpeed", 0);
    Rotation2d ang = Rotation2d.fromRotations(SmartDashboard.getNumber("angle", 0));

    SwerveModuleState state = new SwerveModuleState(sped, ang);

    frontLeftModule.setSwerveState(state);
  }
}
