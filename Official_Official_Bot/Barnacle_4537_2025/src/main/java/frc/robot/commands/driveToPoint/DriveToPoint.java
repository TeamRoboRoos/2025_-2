package frc.robot.commands.driveToPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Drive to an XY point using the OddPod.
 */
public class DriveToPoint extends DriveToPointBase {

    public DriveToPoint(SwerveSubsystem drive, Pose2d position, Measure<DistanceUnit> targetX,
                        Measure<DistanceUnit> targetY,
                        Measure<AngleUnit> targetDegrees, double speed, double tolerance) {
        super(drive, position, targetX, targetY, () -> targetDegrees, speed, tolerance);
    }

    public DriveToPoint(SwerveSubsystem drive, Pose2d position, Measure<DistanceUnit> targetX,
                        Measure<DistanceUnit> targetY,
                        Measure<AngleUnit> targetDegrees, double tolerance) {
        super(drive, position, targetX, targetY, () -> targetDegrees,
                0.75, tolerance);
    }

    public DriveToPoint(SwerveSubsystem drive, Pose2d position, Measure<DistanceUnit> targetX,
                        Measure<DistanceUnit> targetY,
                        Measure<AngleUnit> targetDegrees) {
        super(drive, position, targetX, targetY, () -> targetDegrees);
    }
}