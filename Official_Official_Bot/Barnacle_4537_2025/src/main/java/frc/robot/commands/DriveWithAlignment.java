// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.LinkedList;
import java.util.Queue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveWithAlignment extends Command {
  private SwerveSubsystem swerve;
  private PIDController sidewaysPidController, rotationaPidController, forwardsPidController;
  private boolean shouldFinish = false;

  private Queue<Double> runningAverage;
  private Queue<Double> ummeasureAngleAverage;

  private double last_tx;

  private boolean first_rotated = false;

  private double last_bot_pose_yaw;

  private static final int runningAverageSize = 30;

  private static final int ummeasureAngleSize = 10;

  private boolean initial_sideways_alignment;

  /** Creates a new DriveWithAlignment. */
  public DriveWithAlignment(SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    addRequirements(swerve);

    SmartDashboard.putNumber("limeySideP", 0.06);
    SmartDashboard.putNumber("limeyRotP", 0.03);
    SmartDashboard.putNumber("limeyForP", 0.03);

    SmartDashboard.putNumber("limeySideI", 0.0);
    SmartDashboard.putNumber("limeyRotI", 0.0);
    SmartDashboard.putNumber("limeyForI", 0.0);

    SmartDashboard.putNumber("limeySideD", 0.0);
    SmartDashboard.putNumber("limeyRotD", 0.0);
    SmartDashboard.putNumber("limeyForD", 0.0);

    SmartDashboard.putNumber("threshold", 10);
    SmartDashboard.putNumber("multiplier", 20);

    runningAverage = new LinkedList<Double>();
    ummeasureAngleAverage = new LinkedList<Double>();
    first_rotated = false;

    sidewaysPidController = new PIDController(0.06, 0, 0);
    rotationaPidController = new PIDController(0.03, 0, 0);
    forwardsPidController = new PIDController(0.03, 0, 0);

    initial_sideways_alignment = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    SmartDashboard.putBoolean("running", true);
    SmartDashboard.putBoolean("found_target", false);
    SmartDashboard.putBoolean("finished_alignment", false);
    SmartDashboard.putNumber("primaryTag!", (int) NetworkTableInstance.getDefault().getTable("limelight-limey").getEntry("tid")
    .getInteger(0));

    runningAverage.clear();
    ummeasureAngleAverage.clear();

    shouldFinish = false;

    first_rotated = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double sideways_velocity = 0;
    int limelight_tid = (int) SmartDashboard.getNumber("primaryTag!", -1);

    sidewaysPidController.setP(SmartDashboard.getNumber("limeySideP", 0));
    rotationaPidController.setP(SmartDashboard.getNumber("limeyRotP", 0));
    rotationaPidController.setP(SmartDashboard.getNumber("limeyForP", 0));

    sidewaysPidController.setI(SmartDashboard.getNumber("limeySideI", 0));
    rotationaPidController.setI(SmartDashboard.getNumber("limeyRotI", 0));
    rotationaPidController.setP(SmartDashboard.getNumber("limeyForI", 0));

    sidewaysPidController.setD(SmartDashboard.getNumber("limeySideD", 0));
    rotationaPidController.setD(SmartDashboard.getNumber("limeyRotD", 0));
    rotationaPidController.setP(SmartDashboard.getNumber("limeyForD", 0));

    double tx, ty, ta, tz;
    tx = LimelightHelpers.getTX(LimelightConstants.limelightName);
    ty = LimelightHelpers.getTY(LimelightConstants.limelightName);
    ta = LimelightHelpers.getTA(LimelightConstants.limelightName);
    tz = (NetworkTableInstance.getDefault().getTable("limelight-limey")
        .getEntry("botpose_targetspace").getDoubleArray(new double[6]))[2];

    double[] botpose_targetspace = LimelightHelpers.getBotPose_TargetSpace(LimelightConstants.limelightName);
    double bot_pose_yaw = botpose_targetspace[4];

    // If the limelight sees a tag, all is well, but if it doesnt, go at previous speed but slower
    if (limelight_tid > -1) {
      last_tx = tx;
      sideways_velocity = (sidewaysPidController.calculate(tx, 0));

    } else {
      tx = last_tx;
      sideways_velocity = (sidewaysPidController.calculate(tx, 0)) * 0.7;
    }
    SmartDashboard.putNumber("sideways_velocity", sideways_velocity);

    // Gets an average value for the angle of tag in relation to robot
    runningAverage.add(bot_pose_yaw);
    if (runningAverage.size() > runningAverageSize) {
      runningAverage.poll();
    }

    bot_pose_yaw = 0;
    for (double val : runningAverage) {
      bot_pose_yaw += val;
    }
    bot_pose_yaw /= runningAverage.size();

    SmartDashboard.putNumber("average yaw angle", bot_pose_yaw);

    // double sideways_velocity = tx * -1 * sideways_pid;
    double rotational_velocity = 0;
    double forwards_velocity = 0;

    if (limelight_tid > -1) {
      rotational_velocity = rotationaPidController.calculate(-bot_pose_yaw, 0);
    }

    forwards_velocity = forwardsPidController.calculate(tz, 0) * SmartDashboard.getNumber("multiplier", 1);

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

    // if (first_rotated == false) {
    //   if (Math.abs(tx) < threshold) {
    //     swerve.driveRobotOriented(new ChassisSpeeds(forwards_velocity, 0, 0));
    //     SmartDashboard.putBoolean("under 10", true);
    //   } else {
    //     swerve.driveRobotOriented(new ChassisSpeeds(0, sideways_velocity, 0));
    //     SmartDashboard.putBoolean("under 10", false);
    //   }
    // } else {
    //   swerve.driveRobotOriented(new ChassisSpeeds(forwards_velocity, 0, 0));
    //   SmartDashboard.putBoolean("under 10", true);
    // }

// If its not initially side ways aligned, align it. Then just drive since it will align itself. Only realign sideways
// if it goes way off course
  
  if (initial_sideways_alignment == false) {
    swerve.driveRobotOriented(new ChassisSpeeds(0, sideways_velocity, 0));
    if (Math.abs(tx) < 5) {
      initial_sideways_alignment = true;
    }
  } else {
    if(Math.abs(tx) < 10) {
      swerve.driveRobotOriented(new ChassisSpeeds(forwards_velocity, 0, 0));
    } else {
      swerve.driveRobotOriented(new ChassisSpeeds(0, sideways_velocity, 0));
      initial_sideways_alignment = false;
    }
  }

    // Check if its redy to score
    if (Math.abs(bot_pose_yaw) < LimelightConstants.rotationalTolerance
        && Math.abs(tx) < LimelightConstants.sidewaysTolerance && runningAverage.size() >= runningAverageSize
        && Math.abs(tz) < 1) {

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
