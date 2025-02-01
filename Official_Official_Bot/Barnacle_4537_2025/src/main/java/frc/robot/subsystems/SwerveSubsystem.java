// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.function.Supplier;


import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.imu.NavXSwerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  File directory = new File(Filesystem.getDeployDirectory(), "swerve");
  SwerveDrive swerveDrive;

  StructPublisher<Pose2d> posePublisher;

  public SwerveSubsystem() {

    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.SwerveConstants.maxSpeed,
          new Pose2d(new Translation2d(Meter.of(1),
              Meter.of(4)),
              Rotation2d.fromDegrees(0)));
      swerveDrive.setHeadingCorrection(false);
      // Alternative method if you don't want to supply the conversion factor via JSON
      // files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
      // angleConversionFactor, driveConversionFactor);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    posePublisher = NetworkTableInstance.getDefault()
    .getStructTopic("RobotPose", Pose2d.struct).publish();

    setupPathPlanner();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for (SwerveModule swerveModule : swerveDrive.getModules()) {
      SmartDashboard.putNumber(swerveModule.moduleNumber + " absolute encoder",
          swerveModule.getAbsoluteEncoder().getAbsolutePosition());
      SmartDashboard.putNumber(swerveModule.moduleNumber + " turn motor encoder",
          swerveModule.getAngleMotor().getPosition());
      // SmartDashboard.putBoolean(swerveModule.moduleNumber + " absolute encoder
      // offset?", swerveModule.getAbsoluteEncoder().setAbsoluteEncoderOffset(
      // 34.892578));
    }
    posePublisher.set(swerveDrive.getPose());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public SwerveDrive getSwerveDrive() {

    return swerveDrive;
  }
  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
              swerveDrive::getPose,
              swerveDrive::resetOdometry,
              swerveDrive::getRobotVelocity,
              // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
              (speedsRobotRelative, moduleFeedForwards) -> {
                if (enableFeedforward)
                {
                  swerveDrive.drive(
                          speedsRobotRelative,
                          swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                          moduleFeedForwards.linearForces()
                  );
                } else
                {
                  swerveDrive.setChassisSpeeds(speedsRobotRelative);
                }
              },
              // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
              new PPHolonomicDriveController(
                      // PPHolonomicController is the built in path following controller for holonomic drive trains
                      new PIDConstants(0.0020645, 0.0, 0.0),
                      // Translation PID constants
                      new PIDConstants(0.004, 0.0, 0.0)
                      // Rotation PID constants
              ),
              config,
              // The robot configuration
              () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent())
                {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
              },
              this
              // Reference to this subsystem to set requirements
      );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }

    //Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

  public void resetGyro(Rotation2d angle) {

    swerveDrive.getGyro().setOffset(new Rotation3d(0, 0, 0));
    swerveDrive.getGyro().setOffset(swerveDrive.getGyroRotation3d().plus(new Rotation3d(angle)));
  }

  public void resetPose(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public void stopPlease() {
    swerveDrive.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
  }
}
