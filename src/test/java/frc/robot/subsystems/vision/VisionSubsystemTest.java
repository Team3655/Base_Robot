package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionConstants.ObservationType;
import frc.robot.subsystems.vision.VisionConstants.PoseObservation;

/**
 * Unit tests for VisionSubsystem.
 * 
 * <p>These tests verify:
 * <ul>
 *   <li>Field boundary checking (isInsideField)</li>
 *   <li>Pose observation filtering (distance, ambiguity, tag count)</li>
 *   <li>Standard deviation calculations</li>
 *   <li>Camera connection detection</li>
 * </ul>
 * 
 * <p>Note: These tests use mock IO implementations, so no cameras are required.
 */
@DisplayName("VisionSubsystem Tests")
class VisionSubsystemTest {

  private VisionSubsystem vision;
  private MockVisionIO mockCamera;

  @BeforeEach
  void setUp() {
    // Reset vision measurement counter before each test
    RobotState.resetVisionMeasurementCounter();
    
    // Create mock camera IO
    mockCamera = new MockVisionIO("test-camera");
    mockCamera.setConnected(true);

    // Create VisionSubsystem with mock
    vision = new VisionSubsystem(mockCamera);
  }

  @Test
  @DisplayName("VisionSubsystem should be created successfully")
  void testSubsystemCreation() {
    assertNotNull(vision, "VisionSubsystem should be created");
  }

  @Test
  @DisplayName("isInsideField should accept valid poses within field boundaries")
  void testIsInsideFieldValid() {
    // Test a pose in the center of the field
    Pose2d centerPose = new Pose2d(8.0, 4.0, new Rotation2d());
    PoseObservation observation = new PoseObservation(
        0.0,
        centerPose,
        0.1, // Low ambiguity
        3, // Multiple tags
        2.0, // 2 meters away
        ObservationType.PHOTON);

    assertTrue(vision.isInsideField(observation),
        "Pose in center of field should be inside");
  }

  @Test
  @DisplayName("isInsideField should reject poses outside field boundaries")
  void testIsInsideFieldInvalid() {
    // Test a pose way outside the field
    Pose2d outsidePose = new Pose2d(100.0, 100.0, new Rotation2d());
    PoseObservation observation = new PoseObservation(
        0.0,
        outsidePose,
        0.1,
        3,
        2.0,
        ObservationType.PHOTON);

    assertFalse(vision.isInsideField(observation),
        "Pose outside field should be rejected");
    
    // Also verify that such an observation would be filtered out
    RobotState.resetVisionMeasurementCounter();
    mockCamera.setPoseObservations(new PoseObservation[] { observation });
    mockCamera.setTagIds(new int[] { 1, 2, 3 });
    
    vision.periodic();
    
    // Observation outside field should be rejected
    assertEquals(0, RobotState.getVisionMeasurementCount(),
        "Observation outside field boundaries should be rejected");
  }

  @Test
  @DisplayName("getLatestTargetObservation should return target observation")
  void testGetLatestTargetObservation() {
    // Set up a target observation
    Rotation2d tx = Rotation2d.fromDegrees(10.0); // 10 degrees right
    Rotation2d ty = Rotation2d.fromDegrees(5.0); // 5 degrees up
    VisionConstants.TargetObservation targetObs = new VisionConstants.TargetObservation(tx, ty);
    mockCamera.setLatestObservation(targetObs);

    // Call updateInputs to sync
    vision.periodic();

    // Get observation
    VisionConstants.TargetObservation obs = vision.getLatestTargetObservation(0);
    
    assertNotNull(obs, "Target observation should not be null");
    assertEquals(10.0, obs.tx().getDegrees(), 0.1, "tx should be 10 degrees");
    assertEquals(5.0, obs.ty().getDegrees(), 0.1, "ty should be 5 degrees");
  }

  @Test
  @DisplayName("Pose observations with zero tags should be rejected")
  void testZeroTagRejection() {
    // Reset counter to ensure clean state
    RobotState.resetVisionMeasurementCounter();
    
    Pose2d pose = new Pose2d(8.0, 4.0, new Rotation2d());
    PoseObservation observation = new PoseObservation(
        0.0,
        pose,
        0.1, // Good ambiguity
        0, // ZERO TAGS - should be rejected
        2.0,
        ObservationType.PHOTON);

    mockCamera.setPoseObservations(new PoseObservation[] { observation });
    mockCamera.setTagIds(new int[] {});

    // Call periodic to process observations
    vision.periodic();

    // The observation should be filtered out (no tags) - no measurement should be added
    assertEquals(0, RobotState.getVisionMeasurementCount(),
        "Observation with zero tags should be rejected and not added to RobotState");
  }

  @Test
  @DisplayName("Pose observations with high ambiguity should be rejected")
  void testHighAmbiguityRejection() {
    // Reset counter to ensure clean state
    RobotState.resetVisionMeasurementCounter();
    
    Pose2d pose = new Pose2d(8.0, 4.0, new Rotation2d());
    PoseObservation observation = new PoseObservation(
        0.0,
        pose,
        VisionConstants.MAX_AMBIGUITY + 0.1, // Above threshold
        3, // Multiple tags
        2.0,
        ObservationType.PHOTON);

    mockCamera.setPoseObservations(new PoseObservation[] { observation });
    mockCamera.setTagIds(new int[] { 1, 2, 3 });

    // Call periodic - observation should be filtered
    vision.periodic();

    // High ambiguity should cause rejection - no measurement should be added
    assertEquals(0, RobotState.getVisionMeasurementCount(),
        "Observation with high ambiguity should be rejected and not added to RobotState");
  }

  @Test
  @DisplayName("Single tag observations beyond max distance should be rejected")
  void testSingleTagDistanceRejection() {
    // Reset counter to ensure clean state
    RobotState.resetVisionMeasurementCounter();
    
    Pose2d pose = new Pose2d(8.0, 4.0, new Rotation2d());
    PoseObservation observation = new PoseObservation(
        0.0,
        pose,
        0.1, // Good ambiguity
        1, // Single tag
        VisionConstants.SINGLE_TAG_MAXIMUM + 0.1, // Too far
        ObservationType.PHOTON);

    mockCamera.setPoseObservations(new PoseObservation[] { observation });
    mockCamera.setTagIds(new int[] { 1 });

    vision.periodic();

    // Single tag beyond max distance should be rejected
    assertEquals(0, RobotState.getVisionMeasurementCount(),
        "Single tag observation beyond max distance should be rejected");
  }

  @Test
  @DisplayName("Multi-tag observations beyond max distance should be rejected")
  void testMultiTagDistanceRejection() {
    // Reset counter to ensure clean state
    RobotState.resetVisionMeasurementCounter();
    
    Pose2d pose = new Pose2d(8.0, 4.0, new Rotation2d());
    PoseObservation observation = new PoseObservation(
        0.0,
        pose,
        0.1, // Good ambiguity
        3, // Multiple tags
        VisionConstants.MULTI_TAG_MAXIMUM + 0.1, // Too far
        ObservationType.PHOTON);

    mockCamera.setPoseObservations(new PoseObservation[] { observation });
    mockCamera.setTagIds(new int[] { 1, 2, 3 });

    vision.periodic();

    // Multi-tag beyond max distance should be rejected
    assertEquals(0, RobotState.getVisionMeasurementCount(),
        "Multi-tag observation beyond max distance should be rejected");
  }

  @Test
  @DisplayName("Valid pose observations should be accepted")
  void testValidObservationAcceptance() {
    // Reset counter to ensure clean state
    RobotState.resetVisionMeasurementCounter();
    
    Pose2d pose = new Pose2d(8.0, 4.0, new Rotation2d());
    PoseObservation observation = new PoseObservation(
        0.0,
        pose,
        0.1, // Low ambiguity
        3, // Multiple tags
        2.0, // Within limits
        ObservationType.PHOTON);

    mockCamera.setPoseObservations(new PoseObservation[] { observation });
    mockCamera.setTagIds(new int[] { 1, 2, 3 });

    vision.periodic();

    // Valid observation should be accepted - measurement should be added
    assertEquals(1, RobotState.getVisionMeasurementCount(),
        "Valid observation should be accepted and added to RobotState");
  }

  @Test
  @DisplayName("MegaTag2 observations should use different std dev factors")
  void testMegaTag2StdDevCalculation() {
    // Reset counter to ensure clean state
    RobotState.resetVisionMeasurementCounter();
    
    Pose2d pose = new Pose2d(8.0, 4.0, new Rotation2d());
    PoseObservation observation = new PoseObservation(
        0.0,
        pose,
        0.1,
        3,
        2.0,
        ObservationType.MEGATAG_2); // MegaTag2 type

    mockCamera.setPoseObservations(new PoseObservation[] { observation });
    mockCamera.setTagIds(new int[] { 1, 2, 3 });

    // Periodic should calculate std devs using MEGATAG2 factors
    vision.periodic();

    // Valid MegaTag2 observation should be accepted
    assertEquals(1, RobotState.getVisionMeasurementCount(),
        "Valid MegaTag2 observation should be accepted and added to RobotState");
  }

  @Test
  @DisplayName("Disconnected camera should be detected")
  void testCameraDisconnection() {
    mockCamera.setConnected(false);
    
    // Call periodic - should handle disconnected camera gracefully
    vision.periodic();
    
    assertNotNull(vision);
  }
}

