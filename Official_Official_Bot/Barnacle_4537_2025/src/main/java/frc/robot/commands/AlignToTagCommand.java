// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Deque;
import java.util.LinkedList;
import java.util.Queue;

import javax.naming.LinkLoopException;

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

  private Queue<Double> runningAverage;
  private Queue<Double> ummeasureAngleAverage;

  private double last_tx;
  private double last_bot_pose_yaw;

  private double count = 0;

  private boolean first_rotated = false;

  private static final int runningAverageSize = 30;

  private static final int ummeasureAngleSize = 10;

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
    SmartDashboard.putNumber("threshold", 10);
    SmartDashboard.putNumber("angle_threshold", 10);

    SmartDashboard.putNumber("Close_Rot_P", 10);

    runningAverage = new LinkedList<Double>();
    ummeasureAngleAverage = new LinkedList<Double>();
    first_rotated = false;

    sidewaysPidController = new PIDController(0.06, 0, 0);
    rotationaPidController = new PIDController(0.03, 0, 0);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    SmartDashboard.putBoolean("running", true);
    SmartDashboard.putBoolean("found_target", false);
    SmartDashboard.putBoolean("finished_alignment", false);

    runningAverage.clear();
    ummeasureAngleAverage.clear();

    shouldFinish = false;

    first_rotated = false;
    last_tx = 0;
    last_bot_pose_yaw = 0;
    count = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double sideways_velocity = 0;
    int limelight_tid = (int) NetworkTableInstance.getDefault().getTable("limelight-limey").getEntry("tid")
        .getInteger(0);

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
    double bot_pose_yaw = last_bot_pose_yaw;

    if (limelight_tid > -1) {
      bot_pose_yaw = botpose_targetspace[4];
      last_bot_pose_yaw = bot_pose_yaw;
      last_tx = tx;
      sideways_velocity = (sidewaysPidController.calculate(tx, 0));

    } else {
      tx = last_tx;
      sideways_velocity = (sidewaysPidController.calculate(tx, 0)) * 0.7;

    }
    SmartDashboard.putNumber("sideways_velocity", sideways_velocity);
    runningAverage.add(bot_pose_yaw);
    if (runningAverage.size() > runningAverageSize) {
      runningAverage.poll();
    }

    bot_pose_yaw = 0;
    for (double val : runningAverage) {
      bot_pose_yaw += val;
    }
    bot_pose_yaw /= runningAverage.size();

    ummeasureAngleAverage.add(Math.abs((NetworkTableInstance.getDefault().getTable("limelight-limey")
        .getEntry("botpose_targetspace").getDoubleArray(new double[6]))[4]));
    if (ummeasureAngleAverage.size() > ummeasureAngleSize) {
      ummeasureAngleAverage.poll();
    }

    double angle_thing = 0;
    for (double val : ummeasureAngleAverage) {
      angle_thing += val;
    }
    angle_thing /= ummeasureAngleAverage.size();

    // The average of the last 20 angle measurements of the tag
    SmartDashboard.putNumber("averaged_angle", angle_thing);

    SmartDashboard.putNumber("bot_pose_yaw", bot_pose_yaw);

    // double sideways_velocity = tx * -1 * sideways_pid;
    double rotational_velocity = 0;

    rotational_velocity = rotationaPidController.calculate(-bot_pose_yaw, 0);

    double threshold = SmartDashboard.getNumber("threshold", 10);

    // if (first_rotated == false) {
    // if (angle_thing < threshold) {
    // swerve.driveRobotOriented(new ChassisSpeeds(0, sideways_velocity, 0));
    // SmartDashboard.putBoolean("under 10", true);
    // first_rotated = true;
    // } else {
    // swerve.driveRobotOriented(new ChassisSpeeds(0, 0, rotational_velocity));
    // SmartDashboard.putBoolean("under 10", false);
    // }
    // } else {
    // swerve.driveRobotOriented(new ChassisSpeeds(0, sideways_velocity, 0));
    // SmartDashboard.putBoolean("under 10", true);
    // }
    // System.out.println(SmartDashboard.getNumber("something2", 1));
    // if (Math.abs(SmartDashboard.getNumber("something2", 1)) > threshold) {
    // if (count > 20) {
    // System.out.println("yay!");
    // } else {
    // count += 1;
    // System.out.println("wat");
    // }
    // } else {
    // System.out.println("no");
    // count = 0;
    // }
    // SmartDashboard.putNumber("COUNT", count);
    double angle_threshold = SmartDashboard.getNumber("angle_threshold", 1);
    count += 1;
    System.out.println(count);

    if (count > threshold) {
      swerve.driveRobotOriented(new ChassisSpeeds(0, sideways_velocity, 0));
    } else {

      swerve.driveRobotOriented(new ChassisSpeeds(0, 0, rotational_velocity));
      if (angle_threshold > Math.abs(SmartDashboard.getNumber("something2", 100))) {
        System.out.println("HAPPY");
        SmartDashboard.putBoolean("fr alinged?", true);
        count += threshold;
      } else {

        SmartDashboard.putBoolean("fr alinged?", false);
      }
    }

    if (Math.abs(bot_pose_yaw) < LimelightConstants.rotationalTolerance
        && Math.abs(tx) < LimelightConstants.sidewaysTolerance && runningAverage.size() >= runningAverageSize) {

      SmartDashboard.putBoolean("finished_alignment", true);
      System.out.println("DONE");
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
