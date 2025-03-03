package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CannonSubsystem extends SubsystemBase {
    public SparkMax cannonMotor;
    public Timer cannonTimer;
    public CannonSubsystem(){
        cannonMotor = new SparkMax(20, SparkLowLevel.MotorType.kBrushed);
        cannonTimer = new Timer();
        cannonTimer.reset();
    }
    public Command runCannon(){
        return new StartEndCommand(() -> cannonMotor.set(1), () -> cannonMotor.set(0), this);
    };

}
