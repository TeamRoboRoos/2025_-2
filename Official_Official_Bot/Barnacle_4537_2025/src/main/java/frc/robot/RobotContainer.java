// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignToTagCommand;
import frc.robot.commands.DriveWithAlignment;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import org.opencv.photo.AlignExposures;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  protected final SwerveSubsystem drivebase = new SwerveSubsystem();
  protected final CannonSubsystem m_cannonSubsystem = new CannonSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final LiftSubsystem m_lift = new LiftSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller m_driverController = new CommandPS4Controller(
      OperatorConstants.kDriverControllerPort);
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngularVelocity);
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                  () -> -m_driverController.getRawAxis(1),
                  () -> -m_driverController.getRawAxis(0))
          .withControllerRotationAxis(() -> -m_driverController.getRawAxis(2))
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8);
  // .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
          .withControllerHeadingAxis(() -> -m_driverController.getRawAxis(1),
                  () -> -m_driverController.getRawAxis(0))
          .headingWhile(true);

  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  Command driveFieldOrientedDirectAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
            .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    m_driverController.R2().whileTrue(new AlignToTagCommand(drivebase));
    m_driverController.L2().whileTrue(new DriveWithAlignment(drivebase));
    m_driverController.square().whileTrue(new TurnToAngleCommand(drivebase));

    //m_driverController.triangle().onTrue(new InstantCommand(() -> drivebase.resetGyro(Rotation2d.fromRadians(0))));
    //m_driverController.square()
    //    .onTrue(new InstantCommand(() -> drivebase.resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))));
    m_driverController.triangle().onTrue(new InstantCommand(() -> drivebase.resetGyro(Rotation2d.fromRadians(0))));
    m_driverController.square()
            .onTrue(new InstantCommand(() -> drivebase.resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))));
    m_driverController.L1().onTrue(m_climber.toggleClimberState());
    m_driverController.R1().onTrue(m_lift.toggleLiftState());
    m_driverController.cross().onTrue(m_cannonSubsystem.runCannon());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

}
