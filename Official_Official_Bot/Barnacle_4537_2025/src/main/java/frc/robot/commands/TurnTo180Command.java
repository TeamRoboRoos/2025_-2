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
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnTo180Command extends Command {
  private SwerveSubsystem swerve;

  private PIDController turningPidController;
// Red
  // private int[] AprilTags = {0,0,0,0,0, 45, 0, -45, -135, 179, 135,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
// Blue
  private int[] AprilTags = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-45, 0, 45, 135, -179, -135, 0, 0,0,0,0,0,0,0,0,0,0,0,0};


  private boolean shouldFinish = false;

  private int tid;

  /** Creates a new TurnToAngleCommand. */
  public TurnTo180Command(SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    addRequirements(swerve);

    SmartDashboard.putNumber("turningP", 0.06);
    SmartDashboard.putNumber("turningI", 0.00);
    SmartDashboard.putNumber("turningD", 0.00);
    SmartDashboard.putBoolean("runningCommand", false);


    turningPidController = new PIDController(0.06, 0, 0);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setLEDMode_ForceOn(("limelight-limey"));
    tid = -1;
    tid = (int) SmartDashboard.getNumber("primaryTag!", 0);
    

    shouldFinish = false;
    if (tid == -1) {
      shouldFinish = true;

    }
    SmartDashboard.putNumber("chosen_id", tid);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    turningPidController.setP(SmartDashboard.getNumber("turningP", 0));
    turningPidController.setI(SmartDashboard.getNumber("turningI", 0));
    turningPidController.setD(SmartDashboard.getNumber("turningD", 0));
    
    int desired_angle = 180;
    
    double rotational_velocity = turningPidController.calculate(SmartDashboard.getNumber("Gyro", 0), desired_angle);

    
    SmartDashboard.putNumber("desired_angle", desired_angle);


    swerve.driveRobotOriented(new ChassisSpeeds(0, 0, -rotational_velocity));

    if ((desired_angle - 5) < SmartDashboard.getNumber("Gyro", 0) + 0
        && SmartDashboard.getNumber("Gyro", 0) + 0 < (desired_angle + 5)) {
      shouldFinish = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LimelightHelpers.setLEDMode_ForceOff("limelight-limey");
    
    SmartDashboard.putNumber("chosen_id", -1);
    
    SmartDashboard.putBoolean("runningCommand", false);
    swerve.stopPlease();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shouldFinish;
  }
}
