package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftSubsystem extends SubsystemBase {
    private DoubleSolenoid liftSolenoid;
    private Compressor compressor;

    public LiftSubsystem(){
        liftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 13, 12);
        compressor = new Compressor(1, PneumaticsModuleType.REVPH);
    }
    public Command toggleLiftState(){
        return runOnce(()->liftSolenoid.toggle());
    }
}
