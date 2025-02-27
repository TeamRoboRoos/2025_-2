// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbCommand extends Command {

    // Compressor connected to a PH with a default CAN ID (1)
  int COMPRESSOR_ID = 0;
  private final Compressor m_compressor = new Compressor(COMPRESSOR_ID,PneumaticsModuleType.REVPH);

    // DoubleSolenoid corresponds to a double solenoid.
  // In this case, it's connected to channels 1 and 2 of a PH with the default CAN ID.
  int SOLENOID_ID = 0;
  private final DoubleSolenoid m_doubleSolenoid =
      new DoubleSolenoid(SOLENOID_ID,PneumaticsModuleType.REVPH, 1, 2);

  /** Creates a new ClimbCommand. */
  public ClimbCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_doubleSolenoid.toggle();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
