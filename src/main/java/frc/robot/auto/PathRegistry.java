package frc.robot.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
//import frc.robot.subsystems.util.BSLogger;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;

/**
 * Registry for paths to be used in autonomous mode
 * The swerve base and AutoBuilder must be instantiated before this class is
 * used
 */
public class PathRegistry {
    private final Map<String, LinkedList<PathPlannerAuto>> pathMap;

    public PathRegistry() {
        pathMap = new HashMap<>();
    }

    public void registerPath(String pathName) {
        LinkedList<PathPlannerAuto> registeredPaths = pathMap.getOrDefault(pathName, new LinkedList<>());
        registeredPaths.add(create(pathName));
        pathMap.put(pathName, registeredPaths);
    }

    private PathPlannerAuto create(String pathName) {
        try {
            return new PathPlannerAuto(pathName);
        } catch (Exception e) {
            String message = "[PathRegistry]Error while trying to register an auto path named %s: %s"
                    .formatted(pathName, e);
            //BSLogger.log("PathRegistry", message);
            throw new RuntimeException(message);
        }
    }

    public PathPlannerAuto getPath(String pathName) {
        if (pathMap.containsKey(pathName) && pathMap.get(pathName).peek() != null) {
            return pathMap.get(pathName).pop();
        } else {
//            BSLogger.log("PathRegistry", "Path %s not found, creating new".formatted(pathName));
            return create(pathName);
        }
    }

    public boolean hasPath(String pathName) {
        return pathMap.containsKey(pathName);
    }

    public void registerPaths(String... paths) {
        for (String path : paths) {
            registerPath(path);
        }
    }
}