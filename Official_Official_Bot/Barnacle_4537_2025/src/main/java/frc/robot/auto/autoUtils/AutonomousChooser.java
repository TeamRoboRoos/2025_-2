package frc.robot.auto.autoUtils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShuffleboardTabs;
import java.util.List;
import java.util.Optional;

/**
 * Shuffleboard Autonomous Chooser
 */
public class AutonomousChooser {

    private final SendableChooser<GenericAuto> autos = new SendableChooser<>();

    public AutonomousChooser(List<GenericAuto> autos) {
        // Pick option 1 as default, then add the rest in following
        if (!autos.isEmpty()) {
            this.autos.setDefaultOption(autos.get(0).getName(), autos.get(0));
        }

        for (GenericAuto auto : autos) {
            this.autos.addOption(auto.getName(), auto);
        }

        ShuffleboardTabs.AUTONOMOUS.add("Selected Autonomous", this.autos)
                .withSize(8, 2)
                .withPosition(0, 0);
        ShuffleboardTabs.AUTONOMOUS.addString("Autonomous Description",
                () -> this.autos.getSelected().getDescription()).withSize(8,
                2).withPosition(0, 2);
        Shuffleboard.update();
    }


    public Command getSelected() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return (alliance.get() == DriverStation.Alliance.Blue) ? autos.getSelected().getBlueAuto()
                    : autos.getSelected().getRedAuto();
        }
        return autos.getSelected().getBlueAuto();
    }
}