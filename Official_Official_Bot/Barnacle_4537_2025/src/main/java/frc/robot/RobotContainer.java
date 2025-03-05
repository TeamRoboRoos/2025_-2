// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
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
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.CanonRelease;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.TurnTo0Command;
import frc.robot.commands.TurnTo180Command;
import frc.robot.subsystems.AlgaeRemover;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.PneumaticsHelper;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import org.opencv.photo.AlignExposures;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
  private final PneumaticsHelper pneumaticsHelper = new PneumaticsHelper();
  private final AlgaeRemover m_AlgaeRemover = new AlgaeRemover();
  private final CanonRelease m_CanonRelease = new CanonRelease();
  
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller m_driverController = new CommandPS4Controller(
      OperatorConstants.kDriverControllerPort);
   /*private final CommandPS4Controller m_OperatorController = new CommandPS4Controller(
      OperatorConstants.kOperatorControllerPort); */
  private final CommandGenericHID m_OperatorController = new CommandGenericHID(
      OperatorConstants.kOperatorControllerPort);
  
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    NamedCommands.registerCommand("cannonGoBrr", m_cannonSubsystem.runCannon());
    NamedCommands.registerCommand("armGoBrr", m_lift.toggleLiftState());
    NamedCommands.registerCommand("stopCannon", m_cannonSubsystem.runCannon());
    NamedCommands.registerCommand("faceForwards", new TurnTo180Command(drivebase));
    // NamedCommands.registerCommand("deploy", m_AlgaeRemover.deployAlgaeRemover());
    // NamedCommands.registerCommand("algaeGoBrr", m_AlgaeRemover.runAlgaeRemoverMotor());
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngularVelocity);
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                  () -> -m_driverController.getRawAxis(1) * 0.5,
                  () -> -m_driverController.getRawAxis(0) * 0.5)
          .withControllerRotationAxis(() -> -m_driverController.getRawAxis(2) * 0.5)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8);

  SwerveInputStream drivePrecisionMode = SwerveInputStream.of(drivebase.getSwerveDrive(),
                  () -> -m_driverController.getRawAxis(1) * 0.2,
                  () -> -m_driverController.getRawAxis(0) *0.2)
          .withControllerRotationAxis(() -> -m_driverController.getRawAxis(2) * 0.2)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8);

  SwerveInputStream driveUltraPrecisionMode = SwerveInputStream.of(drivebase.getSwerveDrive(),
                  () -> -m_driverController.getRawAxis(1) ,
                  () -> -m_driverController.getRawAxis(0) )
          .withControllerRotationAxis(() -> -m_driverController.getRawAxis(2) )
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8);

  // .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(() -> -m_driverController.getRawAxis(1),
          () -> -m_driverController.getRawAxis(0))
      .headingWhile(true);

  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  Command driveFieldOrientedDirectAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  
  Command driveFieldOrientedPrecisionMode = drivebase.driveFieldOriented(drivePrecisionMode);
  Command driveFieldOrientedUltraPrecisionMode = drivebase.driveFieldOriented(driveUltraPrecisionMode);

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
    m_driverController.R1().whileTrue(new AlignToTagCommand(drivebase));
    m_driverController.R2().whileTrue(new DriveWithAlignment(drivebase));
    m_driverController.cross().whileTrue(new TurnToAngleCommand(drivebase));
    m_driverController.square().whileTrue(new TurnTo0Command(drivebase));
    m_driverController.triangle().onTrue(new InstantCommand(() -> drivebase.resetGyro(Rotation2d.fromDegrees(0))));
    m_driverController.L1().whileTrue(driveFieldOrientedPrecisionMode);
    m_driverController.L2().whileTrue(driveFieldOrientedUltraPrecisionMode);

    // m_OperatorController.povDown().onTrue(m_climber.toggleClimberState());
    // m_OperatorController.R1().onTrue(m_lift.toggleLiftState());
    // m_OperatorController.L1().whileTrue(m_AlgaeRemover.runAlgaeRemoverMotor());
    // m_OperatorController.R2().whileTrue(m_cannonSubsystem.runCannon());

    // m_OperatorController.cross().onTrue(m_AlgaeRemover.deployAlgaeRemover());
    
    // m_OperatorController.circle().onTrue(m_CanonRelease.deployAlgaeRemover());

    // m_OperatorController.L2().whileTrue(m_cannonSubsystem.loadCannon());

    m_OperatorController.button(10).onTrue(m_climber.toggleClimberState());
    m_OperatorController.button(1).onTrue(m_lift.toggleLiftState());
    m_OperatorController.button(5).whileTrue(m_AlgaeRemover.runAlgaeRemoverMotor());

    m_OperatorController.button(4).whileTrue(m_cannonSubsystem.runCannon());
    // m_OperatorController.button(3).whileTrue(m_cannonSubsystem.backItUp());

    m_OperatorController.button(6).onTrue(m_AlgaeRemover.deployAlgaeRemover());
    
    m_OperatorController.button(9).onTrue(m_CanonRelease.deployAlgaeRemover());

    m_OperatorController.button(2).whileTrue(m_cannonSubsystem.loadCannon());
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
  public SwerveSubsystem getDrivebase(){
    return drivebase;
  }
}
