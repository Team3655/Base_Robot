package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Central configuration class for robot-wide constants and settings.
 * 
 * <p>This class holds all robot-wide numerical, boolean, and configuration constants.
 * It should <b>only</b> contain constants - no functional code. All constants should
 * be declared as {@code public static final}.
 * 
 * <p><b>Key Configuration:</b>
 * <ul>
 *   <li>{@link #currentMode}: Robot mode (REAL, SIM, REPLAY) - auto-detected for REAL</li>
 *   <li>{@link #currentDriver}: Driver control scheme (MAIN, PROGRAMMING, MACBOOK)</li>
 *   <li>{@link #currentRobot}: Robot type (COMPBOT, PROTOBOT) for different hardware configs</li>
 *   <li>{@link #CANIVORE_NAME}: CAN bus name for CTRE devices</li>
 * </ul>
 * 
 * <p><b>Best Practices:</b>
 * <ul>
 *   <li>Use enums for related constants (Mode, Driver, RobotType)</li>
 *   <li>Keep subsystem-specific constants in their own Constants classes (e.g., DriveConstants)</li>
 *   <li>Don't put functional code here - only constants</li>
 *   <li>Consider statically importing this class to reduce verbosity</li>
 * </ul>
 * 
 * <p><b>Usage:</b>
 * <pre>{@code
 * // Check robot mode
 * if (Constants.currentMode == Constants.Mode.REAL) {
 *   // Real robot code
 * }
 * 
 * // Use driver-specific controls
 * switch (Constants.currentDriver) {
 *   case MAIN: // Competition driver
 *   case PROGRAMMING: // Development
 * }
 * }</pre>
 */
public final class Constants {

    public static final String CANIVORE_NAME = "ctre";
    public static final CANBus CANIVORE = new CANBus(CANIVORE_NAME);

    /** Simulation mode setting. Change this to Mode.SIM to force simulation mode. */
    public static final Mode simMode = Mode.SIM;

    /**
     * Current robot mode - automatically detected for real robots.
     * 
     * <p>This constant determines which IO implementations are used:
     * <ul>
     *   <li>REAL: Hardware IO implementations (auto-detected when on real robot)</li>
     *   <li>SIM: Simulation IO implementations (when simMode is set)</li>
     *   <li>REPLAY: Empty IO implementations for log replay</li>
     * </ul>
     * 
     * <p>On a real robot, this will always be REAL (cannot be overridden for safety).
     * In simulation, this uses the value of {@link #simMode}.
     */
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    /**
     * Current driver control scheme.
     * 
     * <p>This determines which control mappings are used in {@link RobotContainer#configureButtonBindings()}.
     * Change this to match your current driver or development environment.
     */
    public static final Driver currentDriver = Driver.MACBOOK;

    /**
     * Driver control scheme options.
     * 
     * <ul>
     *   <li>MAIN: Competition driver using CommandNXT controllers</li>
     *   <li>PROGRAMMING: Development using Xbox controllers</li>
     *   <li>MACBOOK: Simulation/testing on MacBook with different axis mappings</li>
     * </ul>
     */
    public static enum Driver {
        MAIN,
        PROGRAMMING,
        MACBOOK,
    }

    /**
     * Current robot type (for robots with different hardware configurations).
     * 
     * <p>This is used to select different constants (e.g., gear ratios, masses) for
     * different robot builds (competition bot vs. prototype bot).
     */
    public static final RobotType currentRobot = RobotType.COMPBOT;

    /**
     * Robot type options for different hardware configurations.
     */
    public static enum RobotType {
        COMPBOT,
        PROTOBOT
    }

}
