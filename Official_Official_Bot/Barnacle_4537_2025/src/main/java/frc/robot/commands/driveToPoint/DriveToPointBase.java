package frc.robot.commands.driveToPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

/**
 * Drive to an XY point using the OddPod.
 */
public class DriveToPointBase extends Command {

    private final Pose2d position;
    private final SwerveSubsystem drivebase;
    private final Measure<DistanceUnit> targetX;
    private final Measure<DistanceUnit> targetY;
    private final Supplier<Measure<AngleUnit>> targetDegrees;
    private final PIDController autoPID;
    private final double tolerance;
    private final double speed;

    public DriveToPointBase(SwerveSubsystem drive, Pose2d position, Measure<DistanceUnit> targetX,
                            Measure<DistanceUnit> targetY,
                            Supplier<Measure<AngleUnit>> targetDegrees, double speed, double tolerance) {
        this.drivebase = drive;
        this.position = position;
        this.targetX = targetX;
        this.targetY = targetY;
        this.targetDegrees = targetDegrees;
        this.speed = speed;
        this.tolerance = tolerance;
        autoPID = new PIDController(
                0.037272, 0, 0);
        this.addRequirements(drive);
    }

    public DriveToPointBase(SwerveSubsystem drive, Pose2d position, Measure<DistanceUnit> targetX,
                            Measure<DistanceUnit> targetY,
                            Supplier<Measure<AngleUnit>> targetDegrees) {
        this(drive, position, targetX, targetY, targetDegrees,
                0.75, 100);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double targetPowerX = autoPID.calculate(position.getMeasureX().in(Meters), targetX.in(Meters));
        double targetPowerY = autoPID.calculate(position.getMeasureY().in(Meters), targetY.in(Meters));

        double rotateError = (
                ((((targetDegrees.get().in(Degrees) - drivebase.getAHRS().getAngle()) % 360) + 540) % 360)
                        - 180);
        double targetPowerDegrees = 0.55 * rotateError;

        double ratio = speed
                / Math.abs(Math.sqrt(Math.pow(targetPowerX, 2) + Math.pow(targetPowerY, 2)));

        if (ratio < 1) {
            targetPowerX *= ratio;
            targetPowerY *= ratio;
        }

        targetPowerDegrees = Math.min(
                0.3,
                Math.abs(targetPowerDegrees)) * Math.signum(targetPowerDegrees);

        drivebase.driveFieldOriented(ChassisSpeeds.fromFieldRelativeSpeeds(targetPowerX, targetPowerY, targetPowerDegrees, drivebase.getAHRSAngle()));
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.stopPlease();
    }

    // Will end when the OddPod is within 100mm of the target, and gyro within 5 degrees, for tolerance
    @Override
    public boolean isFinished() {
        double num = (Math.round(
                (drivebase.getAHRS().getAngle() / 360) - targetDegrees.get().in(Rotations)));

        return Math.abs(position.getX() - (targetX).in(Millimeters)) < tolerance
                && Math.abs(position.getY() - (targetY).in(Millimeters)) < tolerance
                && Math.abs(drivebase.getAHRS().getAngle() - (targetDegrees.get().in(Degrees) + num * 360))
                < 5;
    }
}