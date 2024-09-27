package frc.robot.config;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.MappingJsonFactory;
//import frc.robot.subsystems.util.BSLogger;

import java.io.*;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

/**
 * Manages loading of robot configurations from a JSON file. Falls back to a default configuration
 * if the specified configuration file cannot be loaded.
 */
public class RobotConfiguration {

    // Static variable for storing the single instance of RobotConfiguration
    private static RobotConfiguration instance;

    // Primary accessor for robot configuration information
    public static RobotConfig config;

    // Path to the file containing the name of the selected robot configuration
    private static final String DEFAULT_CONFIG_FILENAME = "/home/lvuser/robot-config.txt";

    // Default path to the configurations file on the RoboRio
    private static final String CONFIG_FILE_PATH = "/home/lvuser/deploy/config/robot-configs.json";

    // Override System.env name for selected configuration
    static final String SELECTED_CONFIG_ENV = "SELECTED_CONFIG_NAME";

    // Wrap System.env for testing
    private static final Map<String, String> environment = new HashMap<>(System.getenv());

    /**
     * Initializes the robot configuration with the default file path.
     */
    public static synchronized void initialize() {
        if (instance == null) {
            initialize(CONFIG_FILE_PATH);
        }
    }

    /**
     * Initializes the robot configuration with a specified file path.
     * Ensures that only one instance of RobotConfiguration is created (singleton pattern).
     *
     * @param filename Path to the configuration file.
     */
    public static synchronized void initialize(String filename) {
        if (instance == null) {
            instance = new RobotConfiguration(filename);
        }
    }

    /**
     * Private constructor to prevent instantiation from outside the class.
     * Reads the selected configuration name and loads the corresponding configuration.
     *
     * @param filename Path to the configuration file.
     */
    private RobotConfiguration(String filename) {
        config = readSelectedConfigurationName()
                .flatMap(configName -> loadConfiguration(configName, filename))
                .orElseGet(RobotConfig::createDefaultConfig);
    }

    /**
     * Attempts to load the robot configuration from a specified file.
     *
     * @param configurationName The name of the configuration to load.
     * @param filePath          The path to the configuration file.
     * @return An Optional containing the loaded RobotConfig, or an empty Optional if loading fails.
     */
    private static Optional<RobotConfig> loadConfiguration(String configurationName, String filePath) {
        ObjectMapper mapper = new ObjectMapper(new MappingJsonFactory());
        try {
            File configFile = new File(filePath);
            RobotConfigurationOptions configurations = mapper.readValue(configFile, RobotConfigurationOptions.class);
            var configuration = configurations.getConfiguration(configurationName);
            //BSLogger.log("RobotConfiguration", "Loaded configuration for name: " + configurationName);
            return configuration;
        } catch (IOException e) {
            System.err.println("Failed to load configuration from " + filePath + ", using default configuration.");
            return Optional.empty();
        }
    }

    /**
     * Reads the name of the selected configuration from the default configuration file.
     *
     * @return An Optional containing the name of the selected configuration, or an empty Optional if not found.
     */
    private static Optional<String> readSelectedConfigurationName() {
        String returnValue = null; // Declare a final variable for the return value

        String envSelectedName = RobotConfiguration.environment.get(SELECTED_CONFIG_ENV);
        if (envSelectedName != null && !envSelectedName.isBlank()) {
            //BSLogger.log("RobotConfiguration", "Using ENV selected configuration name: " + envSelectedName);
            returnValue = envSelectedName;
        } else {
            try (BufferedReader reader = new BufferedReader(new FileReader(DEFAULT_CONFIG_FILENAME))) {
                String potentialSelected = reader.readLine();
                if (potentialSelected != null && !potentialSelected.isBlank()) {
                    //BSLogger.log("RobotConfiguration", "Using FILE selected configuration name: " + potentialSelected);
                    returnValue = potentialSelected;
                }
            } catch (IOException e) {
                //BSLogger.log("RobotConfiguration", "No configuration file found at: " + DEFAULT_CONFIG_FILENAME + ", using default config");
            }
        }
        return Optional.ofNullable(returnValue);
    }

    /**
     * Utility method to allow manipulation of the environment only for unit tests
     *
     * @return the environment map used by RobotConfiguration
     */
    static Map<String, String> getEnvironmentVariables() {
        return environment;
    }
}
