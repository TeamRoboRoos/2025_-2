package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private DoubleSolenoid climberSolenoid;

    public ClimberSubsystem(){
        climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 14, 15);
        climberSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
    public Command toggleClimberState(){
        return runOnce(()-> climberSolenoid.toggle());
    }
    public Subsystem getClimber(){
        return this;
    }
}
