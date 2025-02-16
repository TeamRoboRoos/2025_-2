package frc.robot.auto.autoUtils;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public abstract class GenericAuto {

    private String name = "", description = "";
    private Supplier<Command> redAutoCommand, blueAutoCommand;

    public final void addName(String name) {
        this.name = name;
    }

    public final void addDescription(String description) {
        this.description = description;
    }

    public final void addRedAuto(Supplier<Command> command) {
        redAutoCommand = command;
    }

    public final void addBlueAuto(Supplier<Command> command) {
        blueAutoCommand = command;
    }

    public final Command getRedAuto() {
        assert redAutoCommand != null : "Red auto does not exist in" + name;
        return redAutoCommand.get().withName("Red " + name);
    }

    public final Command getBlueAuto() {
        assert blueAutoCommand != null : "Blue auto does not exist in" + name;
        return blueAutoCommand.get().withName("Blue " + name);
    }

    public final String getName() {
        return name;
    }

    public final String getDescription() {
        return description;
    }
}