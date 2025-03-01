package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CannonSubsystem extends SubsystemBase {
    public SparkMax cannonMotor;
    public Timer cannonTimer;
    public CannonSubsystem(){
        cannonMotor = new SparkMax(30, SparkLowLevel.MotorType.kBrushed);
        cannonTimer = new Timer();
        cannonTimer.reset();
    }
    public Command runCannon(){
        return new Command() {
            {
                addRequirements(this.getRequirements());

            }
            @Override
            public void initialize(){
                cannonTimer.reset();
                cannonTimer.start();
                System.out.println("timer started");
            }
            @Override
            public void execute(){
                cannonMotor.set(0.75);
                System.out.println("cannon running");
            }
            @Override
            public void end(boolean interrupted){
                cannonMotor.set(0);
                System.out.println("cannon stopped");
                cannonTimer.stop();
                System.out.println("timer stopped");
            }
            @Override
            public boolean isFinished(){
                System.out.println("time up");
                return cannonTimer.hasElapsed(1);
            }
    };}

    public enum CannonState {
        RUNNING,
        OFF,
        RUNNING_REVERSED
    }
}
