package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Volts;

import java.io.IOException;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import frc.robot.RobotState.OdometryMeasurement;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.PathPlannerUtil;

/**
 * Subsystem for managing the swerve drive base.
 * 
 * <p>
 * This subsystem controls a 4-module swerve drive using the IO layer pattern.
 * It receives IO implementations (real hardware, simulation, or replay) and
 * uses them
 * to control the drive base without knowing the specific hardware details.
 * 
 * <p>
 * <b>Key Features:</b>
 * <ul>
 * <li>4 swerve modules (FL, FR, BL, BR) with independent control</li>
 * <li>Gyro integration for heading</li>
 * <li>PathPlanner AutoBuilder integration for autonomous</li>
 * <li>SysId routines for drive characterization</li>
 * <li>High-frequency odometry updates (250Hz via PhoenixOdometryThread)</li>
 * <li>Odometry data sent to RobotState for pose estimation</li>
 * </ul>
 * 
 * <p>
 * <b>IO Layer Pattern:</b> This subsystem uses {@link GyroIO} and
 * {@link ModuleIO}
 * interfaces. The actual implementations are provided by {@link RobotContainer}
 * based
 * on the robot mode (REAL, SIM, REPLAY). This makes the subsystem
 * hardware-agnostic.
 * 
 * <p>
 * <b>Odometry:</b> The subsystem updates {@link RobotState} with odometry
 * measurements
 * every periodic cycle. These measurements include:
 * <ul>
 * <li>High-frequency encoder samples (250Hz) from PhoenixOdometryThread</li>
 * <li>Gyro rotation data</li>
 * <li>Module position deltas</li>
 * </ul>
 * 
 * <p>
 * <b>PathPlanner Integration:</b> AutoBuilder is configured in the constructor
 * to enable
 * autonomous path following. Paths created in PathPlanner automatically appear
 * in the
 * autonomous chooser.
 * 
 * <p>
 * <b>Thread Safety:</b> Uses {@link #odometryLock} to prevent race conditions
 * when
 * the odometry thread updates module positions while the main thread reads
 * them.
 * 
 * <p>
 * <b>Usage:</b>
 * 
 * <pre>{@code
 * // Set robot velocity (field-relative)
 * drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
 *     new ChassisSpeeds(vx, vy, omega),
 *     robotRotation));
 * 
 * // Stop robot
 * drive.stop();
 * 
 * // Stop with X-pattern (defense mode)
 * drive.stopWithX();
 * }</pre>
 */
public class DriveSubsystem extends SubsystemBase {

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;

  /**
   * Constructs a DriveSubsystem with the specified IO implementations.
   * 
   * <p>
   * This constructor:
   * <ol>
   * <li>Initializes 4 swerve modules with their IO implementations</li>
   * <li>Starts the PhoenixOdometryThread for high-frequency odometry</li>
   * <li>Configures PathPlanner AutoBuilder for autonomous</li>
   * <li>Sets up SysId routines for characterization</li>
   * </ol>
   * 
   * <p>
   * <b>IO Implementations:</b> The IO implementations are provided by
   * {@link RobotContainer}
   * based on robot mode. This demonstrates the IO layer pattern - the subsystem
   * doesn't
   * know or care which implementation it receives.
   * 
   * @param gyroIO     Gyro IO implementation (real hardware, sim, or replay)
   * @param flModuleIO Front-left module IO implementation
   * @param frModuleIO Front-right module IO implementation
   * @param blModuleIO Back-left module IO implementation
   * @param brModuleIO Back-right module IO implementation
   */
  public DriveSubsystem(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);
    final double DRIVE_GEAR_RATIO = DriveConstants.DRIVE_GEAR_RATIO;

    // Start threads (no-op for each if no signals have been created)
    PhoenixOdometryThread.getInstance().start();

    ModuleConfig moduleConfig = new ModuleConfig(
        DriveConstants.WHEEL_RADIUS,
        DriveConstants.MAX_LINEAR_SPEED,
        DriveConstants.WHEEL_COF,
        DCMotor.getKrakenX60Foc(1),
        DRIVE_GEAR_RATIO,
        DriveConstants.DRIVE_CURRENT_LIMIT,
        1);

    RobotConfig config = new RobotConfig(
        DriveConstants.ROBOT_MASS_KG,
        DriveConstants.ROBOT_MOI,
        moduleConfig,
        DriveConstants.moduleTranslations);

    try {
      PathPlannerUtil.writeSettings(config, moduleConfig, DRIVE_GEAR_RATIO);
      RobotConfig.fromGUISettings().hasValidConfig();
    } catch (IOException | ParseException e) {
      e.printStackTrace();
    }

    AutoBuilder.configure(
        RobotState.getInstance()::getEstimatedPose, // Robot pose supplier
        RobotState.getInstance()::resetPose, // Method to reset odometry
        this::getChassisSpeeds, // ChassisSpeeds supplier
        this::runVelocity, // Runs robot given chassis speeds
        new PPHolonomicDriveController(
            new PIDConstants(7.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        config,
        () -> {
          // allows path to be flipped for red and blue alliance
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);

    // Allows AdvantageKit to interface with PathPlanner
    Pathfinding.setPathfinder(new LocalADStarAK());
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });

    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> runCharacterization(voltage.in(Volts)),
            null,
            this));
  }

  /**
   * Called every robot periodic cycle (every 20ms).
   * 
   * <p>
   * This method:
   * <ol>
   * <li>Updates inputs from all IO implementations (gyro and modules)</li>
   * <li>Processes inputs for AdvantageKit logging</li>
   * <li>Runs module periodic methods (control loops)</li>
   * <li>Stops modules when robot is disabled</li>
   * <li>Updates RobotState with odometry measurements</li>
   * </ol>
   * 
   * <p>
   * <b>Thread Safety:</b> Uses {@link #odometryLock} to prevent race conditions
   * when reading odometry data that may be updated by the odometry thread.
   * 
   * <p>
   * <b>Odometry Updates:</b> Processes multiple odometry samples per cycle (from
   * high-frequency thread) and sends them to RobotState for pose estimation.
   */
  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();

    Logger.processInputs("Inputs/Drive/Gyro", gyroInputs);

    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Update odometry
    double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;

    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];

        moduleDeltas[moduleIndex] = new SwerveModulePosition(
            modulePositions[moduleIndex].distanceMeters
                - RobotState.getInstance().lastModulePositions[moduleIndex].distanceMeters,
            modulePositions[moduleIndex].angle);

        RobotState.getInstance().lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      RobotState.getInstance().addOdometryMeasurement(
          new OdometryMeasurement(
              sampleTimestamps[i],
              gyroInputs.connected ? gyroInputs.odometryYawPositions[i] : null,
              moduleDeltas,
              modulePositions));
    }
  }

  /**
   * Runs the drive at the desired chassis velocity.
   * 
   * <p>
   * This method converts chassis speeds to individual module states using swerve
   * kinematics, then sends setpoints to each module. The modules handle their own
   * control loops (turning to target angle, driving at target velocity).
   * 
   * <p>
   * <b>Discretization:</b> Chassis speeds are discretized to account for the
   * 20ms control loop period, improving accuracy at high speeds.
   * 
   * <p>
   * <b>Desaturation:</b> Module speeds are desaturated to ensure no module
   * exceeds
   * the maximum linear speed, maintaining the desired direction while scaling
   * down
   * if necessary.
   * 
   * @param speeds Desired chassis speeds (vx, vy, omega) in m/s and rad/s
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = DriveConstants.kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.MAX_LINEAR_SPEED);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and arranges modules in an X-pattern to resist movement.
   * 
   * <p>
   * This is useful for defense or when you want the robot to resist being pushed.
   * The modules are oriented at 45° angles relative to the robot center, creating
   * an X pattern that maximizes resistance to external forces.
   * 
   * <p>
   * <b>Note:</b> The modules will return to normal orientations the next time
   * a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = DriveConstants.moduleTranslations[i].getAngle();
    }
    DriveConstants.kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /**
   * Returns the module states (turn angles and drive velocities) for all of the
   * modules.
   */
  @AutoLogOutput(key = "Drive/SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "Drive/SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /**
   * Returns the average velocity of the modules in rotations/sec (Phoenix native
   * units).
   */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

}
