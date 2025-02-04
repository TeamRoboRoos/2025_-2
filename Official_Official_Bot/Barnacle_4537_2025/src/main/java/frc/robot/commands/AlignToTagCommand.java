// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Deque;
import java.util.LinkedList;
import java.util.Queue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTagCommand extends Command {
  private SwerveSubsystem swerve;

  private PIDController sidewaysPidController, rotationaPidController;
  private boolean shouldFinish = false;

  private int primaryTag = 0;

  private Queue<Double> runningAverage;

  private static final int runningAverageSize = 20;

  /** Creates a new AlignToTagCommand. */
  public AlignToTagCommand(SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    addRequirements(swerve);

    SmartDashboard.putNumber("limeySideP", 0.06);
    SmartDashboard.putNumber("limeyRotP", 0.03);
    SmartDashboard.putNumber("limeySideI", 0.0);
    SmartDashboard.putNumber("limeyRotI", 0.0);
    SmartDashboard.putNumber("limeySideD", 0.0);
    SmartDashboard.putNumber("limeyRotD", 0.0);
    primaryTag = 0;

    runningAverage = new LinkedList<Double>();

    sidewaysPidController = new PIDController(0.06, 0, 0);
    rotationaPidController = new PIDController(0.03, 0, 0);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shouldFinish = false;
    primaryTag = (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getNumber(0);
    SmartDashboard.putNumber("primaryTag", primaryTag);

    SmartDashboard.putBoolean("running", true);

    runningAverage.clear();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    sidewaysPidController.setP(SmartDashboard.getNumber("limeySideP", 0));
    rotationaPidController.setP(SmartDashboard.getNumber("limeyRotP", 0));
    sidewaysPidController.setI(SmartDashboard.getNumber("limeySideI", 0));
    rotationaPidController.setI(SmartDashboard.getNumber("limeyRotI", 0));
    sidewaysPidController.setD(SmartDashboard.getNumber("limeySideD", 0));
    rotationaPidController.setD(SmartDashboard.getNumber("limeyRotD", 0));

    double tx, ty, ta;
    tx = LimelightHelpers.getTX(LimelightConstants.limelightName);
    ty = LimelightHelpers.getTY(LimelightConstants.limelightName);
    ta = LimelightHelpers.getTA(LimelightConstants.limelightName);

    double[] botpose_targetspace = LimelightHelpers.getBotPose_TargetSpace(LimelightConstants.limelightName);
    double bot_pose_yaw = botpose_targetspace[4];
    runningAverage.add(bot_pose_yaw);
    if (runningAverage.size() > runningAverageSize) {
      runningAverage.poll();
    }

    bot_pose_yaw = 0;
    for (double val : runningAverage) {
      bot_pose_yaw += val;
    }
    bot_pose_yaw /= runningAverage.size();

    SmartDashboard.putNumber("bot_pose_yaw", bot_pose_yaw);

    // double sideways_velocity = tx * -1 * sideways_pid;
    double sideways_velocity = sidewaysPidController.calculate(tx, 0);
    double rotational_velocity = rotationaPidController.calculate(-bot_pose_yaw, 0);

    swerve.driveRobotOriented(new ChassisSpeeds(0, sideways_velocity, rotational_velocity));

    if (Math.abs(bot_pose_yaw) < LimelightConstants.rotationalTolerance
        && Math.abs(tx) < LimelightConstants.sidewaysTolerance && runningAverage.size() > runningAverageSize) {
      shouldFinish = true;
    }

    if ((int) SmartDashboard.getNumber("tid", 0) != primaryTag) {
      shouldFinish = true;
    }
  }

  // Called once the command ends or is interrupted.

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("running", false);
    swerve.stopPlease();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return shouldFinish;
  }
}
