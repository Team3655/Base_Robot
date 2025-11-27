package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveConstants;

/**
 * Singleton class that manages the robot's pose estimation and state.
 * 
 * <p>
 * This class serves as the single source of truth for the robot's pose on the
 * field.
 * It combines odometry (from wheel encoders and gyro) with vision measurements
 * to provide
 * the most accurate pose estimate possible.
 * 
 * <p>
 * <b>Why Singleton?</b> Having a single RobotState instance ensures:
 * <ul>
 * <li>All subsystems use the same pose data (no divergence)</li>
 * <li>Single pose estimator prevents inconsistencies</li>
 * <li>Centralized state management makes debugging easier</li>
 * </ul>
 * 
 * <p>
 * <b>Pose Estimation:</b> Uses two estimators:
 * <ul>
 * <li>{@link SwerveDriveOdometry}: Odometry-only pose (fast, continuous, but
 * drifts)</li>
 * <li>{@link SwerveDrivePoseEstimator}: Fused pose (odometry + vision, more
 * accurate)</li>
 * </ul>
 * 
 * <p>
 * <b>Data Flow:</b>
 * <ol>
 * <li>{@link DriveSubsystem} calls {@link #addOdometryMeasurement()} every
 * periodic cycle</li>
 * <li>{@link VisionSubsystem} calls {@link #addVisionMeasurement()} when vision
 * detects tags</li>
 * <li>Subsystems call {@link #getEstimatedPose()} or {@link #getPose()} to get
 * current pose</li>
 * </ol>
 * 
 * <p>
 * <b>Thread Safety:</b> All public methods are synchronized to prevent race
 * conditions
 * when multiple threads (e.g., odometry thread) update the pose simultaneously.
 * 
 * <p>
 * <b>Usage:</b> Always access via {@link #getInstance()} - never instantiate
 * directly.
 * 
 * <pre>{@code
 * // Get current pose
 * Pose2d pose = RobotState.getInstance().getPose();
 * 
 * // Reset pose
 * RobotState.getInstance().resetPose(new Pose2d(x, y, rotation));
 * 
 * // Zero heading only
 * RobotState.getInstance().zeroHeading();
 * }</pre>
 * 
 * <p>
 * <b>NOTE:</B> The state for every other mechanism on your robot should be
 * stored and accessed here as well. All mechanisms should follow a similar data
 * flow to pose estimation
 */
public class RobotState {

  public record OdometryMeasurement(
      double timestamp,
      Rotation2d gyroRotation,
      SwerveModulePosition[] moduleDeltas,
      SwerveModulePosition[] wheelPositions) {
  }

  public record VisionMeasurement(
      double timestamp,
      Pose2d pose,
      Matrix<N3, N1> stdDevs) {
  }

  private SwerveDriveKinematics kinematics;

  private SwerveDriveOdometry odometry;
  private SwerveDrivePoseEstimator poseEstimator;

  public SwerveModulePosition[] lastModulePositions;
  public Rotation2d rawGyroRotation;

  private static RobotState instance;

  /**
   * Gets the singleton instance of RobotState.
   * 
   * <p>
   * Creates the instance on first call (lazy initialization). Subsequent calls
   * return the same instance.
   * 
   * @return The singleton RobotState instance
   */
  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  private RobotState() {

    lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };

    kinematics = DriveConstants.kinematics;

    odometry = new SwerveDriveOdometry(
        kinematics,
        new Rotation2d(),
        lastModulePositions);

    poseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        new Rotation2d(),
        lastModulePositions,
        new Pose2d(),
        VecBuilder.fill(Units.inchesToMeters(2.0), Units.inchesToMeters(2.0), Units.degreesToRadians(2.0)),
        VecBuilder.fill(0.5, 0.5, 0.5));

    rawGyroRotation = new Rotation2d();
  }

  /**
   * Adds an odometry measurement from the drive subsystem.
   * 
   * <p>
   * This method is called by {@link DriveSubsystem} every periodic cycle with
   * high-frequency encoder and gyro data. It updates both the odometry-only pose
   * and the fused pose estimator.
   * 
   * <p>
   * <b>Thread Safety:</b> This method is synchronized to prevent race conditions
   * when called from multiple threads (e.g., the odometry thread).
   * 
   * @param measurement The odometry measurement containing timestamps, gyro
   *                    rotation,
   *                    module deltas, and wheel positions
   */
  public synchronized void addOdometryMeasurement(OdometryMeasurement measurement) {
    // Update gyro angle
    if (measurement.gyroRotation != null) {
      // Use the real gyro angle
      rawGyroRotation = measurement.gyroRotation;
    } else {
      // Use the angle delta from the kinematics and module deltas
      Twist2d twist = DriveConstants.kinematics.toTwist2d(measurement.moduleDeltas);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    odometry.update(
        rawGyroRotation,
        measurement.wheelPositions);

    poseEstimator.updateWithTime(
        measurement.timestamp,
        rawGyroRotation,
        measurement.wheelPositions);
  }

  /**
   * Adds a vision measurement from the vision subsystem.
   * 
   * <p>
   * This method is called by {@link VisionSubsystem} when AprilTags are detected.
   * The measurement includes uncertainty (standard deviations) which the pose
   * estimator
   * uses to weight the vision data appropriately.
   * 
   * <p>
   * <b>Fusion:</b> The pose estimator automatically combines vision measurements
   * with
   * odometry, giving more weight to measurements with lower uncertainty (closer
   * tags,
   * more tags, etc.).
   * 
   * @param measurement The vision measurement containing timestamp, pose
   *                    estimate, and
   *                    standard deviations (uncertainty)
   */
  public synchronized void addVisionMeasurement(VisionMeasurement measurement) {
    poseEstimator.addVisionMeasurement(
        measurement.pose,
        measurement.timestamp,
        measurement.stdDevs);
  }

  /**
   * Resets the robot's pose to a known position.
   * 
   * <p>
   * This method resets both the odometry-only pose and the fused pose estimator
   * to the specified pose. Use this when:
   * <ul>
   * <li>Starting a match (reset to starting position)</li>
   * <li>Vision detects a known position (e.g., AprilTag)</li>
   * <li>Recovering from pose estimation errors</li>
   * </ul>
   * 
   * @param pose The new pose to set (position and rotation)
   */
  public synchronized void resetPose(Pose2d pose) {
    odometry.resetPosition(
        rawGyroRotation,
        lastModulePositions,
        pose);

    poseEstimator.resetPosition(
        rawGyroRotation,
        lastModulePositions,
        pose);
  }

  /**
   * Zeros the robot's heading (rotation) while preserving its position.
   * 
   * <p>
   * This is useful when you want to reset the robot's orientation to 0° (facing
   * forward) but keep its current X/Y position. Commonly used when:
   * <ul>
   * <li>Driver presses a "zero heading" button</li>
   * <li>Starting autonomous from a known orientation</li>
   * </ul>
   * 
   * <p>
   * This method calls {@link #resetPose(Pose2d)} with the current position but
   * a zero rotation.
   */
  public synchronized void zeroHeading() {
    resetPose(new Pose2d(
        poseEstimator.getEstimatedPosition().getX(),
        poseEstimator.getEstimatedPosition().getY(),
        new Rotation2d()));
  }

  public Rotation2d getRotation() {
    return getOdometryPose().getRotation();
  }

  /**
   * Gets the robot's current pose (position and rotation).
   * 
   * <p>
   * This is the recommended method for getting the robot's pose. It returns a
   * pose with the estimated position (from fused estimator) but uses the odometry
   * rotation for heading (more reliable for field-relative driving).
   * 
   * <p>
   * This pose is automatically logged by AdvantageKit via {@link AutoLogOutput}.
   * 
   * @return The robot's current pose on the field
   */
  @AutoLogOutput(key = "RobotState/Pose")
  public Pose2d getPose() {
    return new Pose2d(
        getEstimatedPose().getTranslation(),
        getRotation());
  }

  /**
   * Gets the robot's estimated pose (fused odometry + vision).
   * 
   * <p>
   * This pose combines odometry and vision measurements for maximum accuracy.
   * It's the best estimate of where the robot is on the field.
   * 
   * <p>
   * This pose is automatically logged by AdvantageKit via {@link AutoLogOutput}.
   * 
   * @return The robot's estimated pose (fused odometry + vision)
   */
  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Gets the robot's odometry-only pose (no vision fusion).
   * 
   * <p>
   * This pose is based solely on wheel encoders and gyro. It's fast and
   * continuous
   * but will drift over time. Useful for debugging or when vision is unavailable.
   * 
   * <p>
   * This pose is automatically logged by AdvantageKit via {@link AutoLogOutput}.
   * 
   * @return The robot's odometry-only pose
   */
  @AutoLogOutput(key = "RobotState/OdometryPose")
  public Pose2d getOdometryPose() {
    return odometry.getPoseMeters();
  }

}
