package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftSubsystem extends SubsystemBase {
    private DoubleSolenoid liftSolenoid;

    public LiftSubsystem(){
        liftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 13, 12);
        liftSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
    public Command toggleLiftState(){
        return runOnce(()->liftSolenoid.toggle());
    }
}
