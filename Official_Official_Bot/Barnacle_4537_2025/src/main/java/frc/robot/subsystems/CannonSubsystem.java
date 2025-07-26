package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CannonSubsystem extends SubsystemBase {
    public DigitalInput intakeSwitch;
    
    public SparkMax cannonMotor;
    public Timer cannonTimer;
    public CannonSubsystem(){
        cannonMotor = new SparkMax(20, SparkLowLevel.MotorType.kBrushed);
        cannonTimer = new Timer();
        cannonTimer.reset();
        intakeSwitch = new DigitalInput(0);
    }
    public Command runCannon(){
        System.out.println("CANONNNN");
        return new StartEndCommand(() -> cannonMotor.set(-1), () -> cannonMotor.set(0), this).withTimeout(0.8);
    };
    public Command AutorunCannon(){
        System.out.println("CANONNNN");
        return new StartEndCommand(() -> cannonMotor.set(-1), () -> cannonMotor.set(0), this).until(() -> intakeSwitch.get()).withTimeout(0.8);
    };
    public Command backItUp(){
        return new StartEndCommand(() -> cannonMotor.set(1), () -> cannonMotor.set(0), this).withTimeout(0.8);
    };
    public Command loadCannon(){
        System.out.println("halooooo");
        return new StartEndCommand(() -> cannonMotor.set(-1), () -> cannonMotor.set(0), this).until(() -> !intakeSwitch.get());
    };

}
