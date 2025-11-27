package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionMeasurement;
import frc.robot.subsystems.vision.VisionConstants.ObservationType;
import frc.robot.subsystems.vision.VisionConstants.PoseObservation;
import frc.robot.subsystems.vision.VisionConstants.TargetObservation;

/**
 * Subsystem for managing vision processing and pose estimation from cameras.
 * 
 * <p>
 * This subsystem processes vision data from one or more cameras to estimate the
 * robot's pose on the field using AprilTags. It filters invalid measurements
 * and
 * adds valid ones to {@link RobotState} for fusion with odometry.
 * 
 * <p>
 * <b>Key Features:</b>
 * <ul>
 * <li>Supports multiple cameras (variable number via constructor)</li>
 * <li>AprilTag detection and pose estimation</li>
 * <li>Pose filtering (rejects invalid measurements)</li>
 * <li>Field boundary checking</li>
 * <li>Standard deviation calculation based on tag distance and count</li>
 * <li>Automatic alerts for disconnected cameras</li>
 * </ul>
 * 
 * <p>
 * <b>IO Layer Pattern:</b> This subsystem uses {@link VisionIO} interfaces.
 * Different implementations (Limelight / PhotonVisionSimulation) can be
 * provided
 * without changing the subsystem code.
 * 
 * <p>
 * <b>Pose Filtering:</b> Vision measurements are filtered based on:
 * <ul>
 * <li>Tag count: Must have at least one tag</li>
 * <li>Ambiguity: Must be below threshold (default 0.25)</li>
 * <li>Distance: Single tags max 3.5m, multi-tags max 7.5m</li>
 * <li>Field bounds: Must be within field boundaries</li>
 * </ul>
 * 
 * <p>
 * <b>Standard Deviations:</b> Calculated based on tag distance and count.
 * Closer tags and more tags result in lower uncertainty (higher trust). The
 * pose
 * estimator uses these to weight vision measurements appropriately.
 * 
 * <p>
 * <b>Data Flow:</b> Valid pose observations are added to {@link RobotState}
 * via {@link RobotState#addVisionMeasurement(VisionMeasurement)}. RobotState
 * then
 * fuses them with odometry for the best pose estimate.
 * 
 * <p>
 * <b>Usage:</b>
 * 
 * <pre>{@code
 * // Get target observation for aiming (not pose estimation)
 * TargetObservation obs = vision.getLatestTargetObservation(0);
 * Rotation2d tx = obs.tx(); // Horizontal angle to target
 * }</pre>
 * 
 * <p>
 * <b>Note:</b> This subsystem doesn't expose commands directly. Vision
 * measurements
 * are automatically processed and added to RobotState. For targeting (not pose
 * estimation),
 * use {@link #getLatestTargetObservation(int)}.
 */
public class VisionSubsystem extends SubsystemBase {

  private final VisionIOInputsAutoLogged[] inputs;
  private final VisionIO[] io;
  private final Alert[] disconnectedAlerts;
  private AprilTagFieldLayout tagLayout;

  /**
   * Constructs a VisionSubsystem with the specified camera IO implementations.
   * 
   * <p>
   * This constructor supports multiple cameras by accepting a variable number of
   * VisionIO implementations. Each camera is processed independently in
   * periodic().
   * 
   * <p>
   * <b>IO Implementations:</b> The IO implementations are provided by
   * {@link RobotContainer}
   * based on robot mode. This demonstrates the IO layer pattern - the subsystem
   * doesn't
   * know which implementation it receives (Limelight, PhotonVision, Simulation,
   * etc.).
   * 
   * <p>
   * <b>AprilTag Layout:</b> Loads the AprilTag field layout for the current game
   * year.
   * Update the field layout constant if using a different year's field.
   * 
   * @param io Variable number of VisionIO implementations (one per camera)
   */
  public VisionSubsystem(VisionIO... io) {
    this.io = io;
    this.inputs = new VisionIOInputsAutoLogged[io.length];

    try {
      tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load april tags :3 !!", null);
    }

    // Create inputs
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < disconnectedAlerts.length; i++) {
      disconnectedAlerts[i] = new Alert(inputs[i].name + " is disconnected", AlertType.kWarning);
    }
  }

  /**
   * Gets the latest target observation from a specific camera.
   * 
   * <p>
   * This method returns targeting data (angles to target) rather than pose
   * estimation
   * data. Use this for aiming at targets, not for pose estimation.
   * 
   * <p>
   * <b>Note:</b> For pose estimation, the subsystem automatically processes
   * observations and adds them to RobotState. This method is only for targeting.
   * 
   * @param cameraID The index of the camera (0-based, matches order in
   *                 constructor)
   * @return The latest target observation with tx and ty angles
   */
  public TargetObservation getLatestTargetObservation(int cameraID) {
    return new TargetObservation(inputs[cameraID].latestObservation.tx(), inputs[cameraID].latestObservation.ty());
  }

  public boolean isInsideField(PoseObservation observation) {
    double slope = Units.inchesToMeters(0.7587);
    double x = observation.pose().getX();
    double y = observation.pose().getY();

    boolean isInsideBlueLeft = (y < slope * x + Units.inchesToMeters(267.2));
    boolean isInsideBlueRight = (y > -slope * x + Units.inchesToMeters(49.95));
    boolean isInsideRedLeft = (y > slope * x - Units.inchesToMeters(474.187));
    boolean isInsideRedRight = (y < -slope * x + Units.inchesToMeters(791.337));
    boolean isInsideRectangle = (x > 0 && x < tagLayout.getFieldLength() && y > 0 && y < tagLayout.getFieldWidth());

    if (isInsideBlueRight && isInsideBlueLeft && isInsideRedLeft && isInsideRedRight && isInsideRectangle) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * Called every robot periodic cycle (every 20ms).
   * 
   * <p>
   * This method:
   * <ol>
   * <li>Updates inputs from all camera IO implementations</li>
   * <li>Checks camera connections and alerts if disconnected</li>
   * <li>Processes pose observations from each camera</li>
   * <li>Filters invalid measurements (distance, ambiguity, field bounds)</li>
   * <li>Calculates standard deviations based on tag distance and count</li>
   * <li>Adds valid measurements to RobotState for pose fusion</li>
   * <li>Logs vision data for debugging</li>
   * </ol>
   * 
   * <p>
   * <b>Pose Processing:</b> Each camera may produce multiple pose observations
   * per cycle. Each observation is validated and processed independently. Valid
   * observations are added to RobotState with calculated uncertainty.
   * 
   * <p>
   * <b>Filtering:</b> Observations are rejected if they:
   * <ul>
   * <li>Have no tags detected</li>
   * <li>Exceed ambiguity threshold</li>
   * <li>Are too far away (single tag: 3.5m, multi-tag: 7.5m)</li>
   * <li>Are outside field boundaries</li>
   * </ul>
   */
  @Override
  public void periodic() {

    // Update inputs
    for (int i = 0; i < inputs.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Inputs/Vision/Camera " + inputs[i].name, inputs[i]);
    }

    for (int i = 0; i < io.length; i++) {
      // Check if camera is disconnected and alert if so
      disconnectedAlerts[i].set(!inputs[i].connected);

      // Logging data
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose2d> robotPoses = new LinkedList<>();
      List<Pose2d> robotPosesAccepted = new LinkedList<>();
      List<Pose2d> robotPosesRejected = new LinkedList<>();

      // Get positions of every tag that is seen
      for (int tagId : inputs[i].tagIds) {
        var tagPose = tagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        } else {
          continue;
        }
      }

      // Go through pose observations and add them to our estimated pose
      for (var observation : inputs[i].poseObservations) {

        // Check if pose should be rejected
        boolean rejectPose = observation.tagCount() > 1 ?

        // Multiple tags in observation
            observation.tagCount() == 0 // Must have at least one tag
                || observation.ambiguity() > VisionConstants.MAX_AMBIGUITY // Must be a trustworthy pose
                || observation.averageTagDistance() > VisionConstants.MULTI_TAG_MAXIMUM // Must not be too far away
                // Must be within the field
                || !isInsideField(observation)

            // Single tag in observation
            : observation.tagCount() == 0 // Must have at least one tag
                || observation.ambiguity() > VisionConstants.MAX_AMBIGUITY // Must be a trustworthy pose
                || observation.averageTagDistance() > VisionConstants.SINGLE_TAG_MAXIMUM // Must not be too far away
                // Must be within the field
                || !isInsideField(observation);

        // Add pose to list of all poses
        robotPoses.add(observation.pose());

        // Add it to its respective list and skip calculation if pose was rejected
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
          continue;
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        double linearStdDev;
        double angularStdDev;

        // Calculate standard deviations
        linearStdDev = VisionConstants.LINEAR_STD_DEV_FACTOR * Math.pow(observation.averageTagDistance(), 2)
            / observation.tagCount();
        angularStdDev = VisionConstants.ANGULAR_STD_DEV_FACTOR * Math.pow(observation.averageTagDistance(), 2)
            / observation.tagCount();

        if (observation.type() == ObservationType.MEGATAG_2) {
          linearStdDev = VisionConstants.MEGATAG2_LINEAR_FACTOR * Math.pow(observation.averageTagDistance(), 2)
              / observation.tagCount();
          angularStdDev = VisionConstants.MEGATAG2_ANGULAR_FACTOR * Math.pow(observation.averageTagDistance(), 2)
              / observation.tagCount();
        }

        // Add the measurement to the poseEstimator
        RobotState.getInstance()
            .addVisionMeasurement(
                new VisionMeasurement(
                    observation.timestamp(),
                    observation.pose(),
                    VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)));
      }

      // Log individual camera data
      Logger.recordOutput(
          "Vision/Cameras/" + inputs[i].name + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Cameras/" + inputs[i].name + "/RobotPoses",
          robotPoses.toArray(new Pose2d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Cameras/" + inputs[i].name + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose2d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Cameras/" + inputs[i].name + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose2d[robotPosesRejected.size()]));
    }
  }
}
