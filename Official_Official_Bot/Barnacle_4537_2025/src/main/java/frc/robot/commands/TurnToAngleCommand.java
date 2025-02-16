// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.reflect.Array;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnToAngleCommand extends Command {
  private Subsystem swerve;

  private PIDController turningPidController;

  private int[] AprilTags = { -135, 135, 90, 0, 0, 45, 0, -45, -135, 180, 135 };

  private boolean shouldFinish = false;

  private int tid;

  /** Creates a new TurnToAngleCommand. */
  public TurnToAngleCommand(SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    addRequirements(swerve);

    SmartDashboard.putNumber("turningP", 0.06);
    SmartDashboard.putNumber("turningI", 0.06);
    SmartDashboard.putNumber("turningD", 0.06);

    turningPidController = new PIDController(0.06, 0, 0);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tid = 0;
    tid = (int) NetworkTableInstance.getDefault().getTable("limelight-limey").getEntry("tid")
        .getInteger(0);

    shouldFinish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    turningPidController.setP(SmartDashboard.getNumber("turningP", 0));
    turningPidController.setI(SmartDashboard.getNumber("turningI", 0));
    turningPidController.setD(SmartDashboard.getNumber("turningD", 0));

    int desired_angle = AprilTags[tid];

    double rotational_velocity = turningPidController.calculate(SmartDashboard.getNumber("Gyro", 0), desired_angle);

    swerve.driveRobotOriented(new ChassisSpeeds(0, 0, rotational_velocity));

    if ((desired_angle - 5) < SmartDashboard.getNumber("Gyro", 0)
        && SmartDashboard.getNumber("Gyro", 0) < (desired_angle + 5)) {
      shouldFinish = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopPlease();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shouldFinish;
  }
}
