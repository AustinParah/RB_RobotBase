package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.HashMap;
import java.util.function.Supplier;

/**
 * Manages the autonomous mode of the robot.
 * <p>
 * Steps to add a new strategy:
 * 1. Create a new command that implements the strategy, most likely extending
 * AutoPathStrategy or SequentialCommandGroup
 * 2. Add a new enum value to AutoStrategies
 * 3. Register the strategy in the registerStrategies method
 * 4. Register the path in the registerAutoPaths method
 * 5. Register any named commands in the registerNamedCommands method
 */
public class AutoManager {

    private static final PathRegistry pathRegistry = new PathRegistry();
    private final SendableChooser<AutoStrategies> chooser = new SendableChooser<>();
    private final HashMap<AutoStrategies, Command> strategyLookup = new HashMap<>();

    public AutoManager() {
    }

    /**
     * Initializes the autonomous manager.
     * Must be called before using the manager.
     * Put in place to work around some static reference ordering issues.
     */
    public void initialize() {
        registerNamedCommands();
        registerAutoPaths();
        registerStrategies();

        // Send selector Dashboard. If it doesn't show in SD, you may need to change the
        // name here.
        SmartDashboard.putData("Auto Selector", chooser);
    }

    /**
     * Returns the command for the given path name
     *
     * @param pathName the name of the path
     * @return the command for the path
     */
    public Command getAutoPath(String pathName) {
        return pathRegistry.getPath(pathName);
    }

    /**
     * Registers the autonomous strategies with the chooser and the lookup map
     */
    private void registerStrategies() {

    }

    /**
     * Registers Named commands for use in PathPlanner GUI
     */
    private void registerNamedCommands() {

    }

    /**
     * Registers the autonomous paths with the path registry
     */
    private void registerAutoPaths() {
        // Tab.registerAutoPaths(pathRegistry);
        // DropShot.registerAutoPaths(pathRegistry);
        // Blaze.registerAutoPaths(pathRegistry);
    }

    // Register the strategy with the chooser and the lookup map
    private void registerStrategy(String displayName, AutoStrategies strategyEnum, Supplier<Command> strategy) {
        registerStrategy(displayName, strategyEnum, strategy, false);
    }

    // Register the strategy with the chooser and the lookup map
    private void registerStrategy(String displayName, AutoStrategies strategyEnum, Supplier<Command> strategy,
                                  boolean isDefault) {
        if (isDefault) {
            chooser.setDefaultOption(displayName, strategyEnum);
        } else {
            chooser.addOption(displayName, strategyEnum);
        }
        strategyLookup.put(strategyEnum, strategy.get());
    }

    /**
     * Returns the currently selected autonomous strategy
     *
     * @return the currently selected autonomous strategy
     */
    public Command getSelectedAutoStrategy() {
        Command selected = null;

        if (strategyLookup.containsKey(chooser.getSelected())) {
            selected = strategyLookup.get(chooser.getSelected());
        } else {
            // Missing autonomous key in lookup map
            selected = Commands.print("[AutoManager] ERROR: Could not find requested auto: " + chooser.getSelected());
        }
        return selected;
    }

    public void showSelectedAuto() {
        var selected = chooser.getSelected();
        SmartDashboard.putString("Selected Auto", (selected != null) ? selected.name() : "Unknown");
    }

    public enum AutoStrategies {
        DebugAuto,
        DebugTestPath,
        DebugTestMultipath,
        DebugCurly,
        DebugWavy,
        DebugTests2,
        DCL_BCL,
        UnderTheBridge,
        DropShot,
        Tab,
        TabStraight,
        CrossWing,
        SL1toSL3,
        BlazeAB,
        BlazeBA,
        BlazeBC,
        BlazeCB,
        BlazeAC
    }

}
