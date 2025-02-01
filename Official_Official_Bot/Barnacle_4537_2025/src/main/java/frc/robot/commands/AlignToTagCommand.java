// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTagCommand extends Command {
  private SwerveSubsystem swerve;

  /** Creates a new AlignToTagCommand. */
  public AlignToTagCommand(SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    addRequirements(swerve);

    SmartDashboard.putNumber("limeyp", 0.06);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("hello");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double pid_thing = SmartDashboard.getNumber("limeyp", 0);

    double tx, ty, ta;
    tx = LimelightHelpers.getTX(LimelightConstants.limelightName);
    ty = LimelightHelpers.getTY(LimelightConstants.limelightName);
    ta = LimelightHelpers.getTA(LimelightConstants.limelightName);

    double cool_variable = tx * -1 * pid_thing;

    swerve.driveFieldOriented(new ChassisSpeeds(0, cool_variable, 0));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopPlease();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
