package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveConstants;

/**
 * Unit tests for validating robot constants.
 * 
 * <p>These tests ensure that constants are physically reasonable and consistent.
 * This helps catch configuration errors early.
 */
@DisplayName("Constants Validation Tests")
class ConstantsTest {

  @Test
  @DisplayName("Track width should be positive")
  void testTrackWidthPositive() {
    assertTrue(Constants.TRACK_WIDTH_X > 0, "Track width X should be positive");
    assertTrue(Constants.TRACK_WIDTH_Y > 0, "Track width Y should be positive");
  }

  @Test
  @DisplayName("Bumper width should be larger than track width")
  void testBumperLargerThanTrack() {
    assertTrue(
        Constants.BUMPER_WIDTH_X >= Constants.TRACK_WIDTH_X,
        "Bumper width X should be >= track width X");
    assertTrue(
        Constants.BUMPER_WIDTH_Y >= Constants.TRACK_WIDTH_Y,
        "Bumper width Y should be >= track width Y");
  }

  @Test
  @DisplayName("Max linear speed should be positive and reasonable")
  void testMaxLinearSpeed() {
    double maxSpeedMps = DriveConstants.MAX_LINEAR_SPEED;
    assertTrue(maxSpeedMps > 0, "Max linear speed should be positive");
    
    // FRC robots typically don't exceed 20 ft/s (~6 m/s)
    double reasonableMax = Units.feetToMeters(25);
    assertTrue(maxSpeedMps < reasonableMax,
        String.format("Max speed %.2f m/s seems unreasonably high", maxSpeedMps));
  }

  @Test
  @DisplayName("Wheel radius should be positive and reasonable")
  void testWheelRadius() {
    double radius = DriveConstants.WHEEL_RADIUS;
    assertTrue(radius > 0, "Wheel radius should be positive");
    
    // Wheel radius should be reasonable (typical swerve wheels are 2-4 inches)
    double minRadius = Units.inchesToMeters(1.0);
    double maxRadius = Units.inchesToMeters(5.0);
    assertTrue(radius >= minRadius && radius <= maxRadius,
        String.format("Wheel radius %.4f m seems unreasonable (expected ~1-5 inches)",
            radius));
  }

  @Test
  @DisplayName("Gear ratios should be positive")
  void testGearRatios() {
    assertTrue(DriveConstants.DRIVE_GEAR_RATIO > 0, "Drive gear ratio should be positive");
    assertTrue(DriveConstants.TURN_GEAR_RATIO > 0, "Turn gear ratio should be positive");
  }

  @Test
  @DisplayName("Robot mass should be positive and reasonable")
  void testRobotMass() {
    double mass = DriveConstants.ROBOT_MASS_KG;
    assertTrue(mass > 0, "Robot mass should be positive");
    
    // FRC robots are limited to ~125 lbs (~57 kg) with bumpers
    double maxMass = Units.lbsToKilograms(150);
    assertTrue(mass <= maxMass,
        String.format("Robot mass %.2f kg seems unreasonably high", mass));
  }

  @Test
  @DisplayName("Vision maximum distances should be positive")
  void testVisionMaxDistances() {
    assertTrue(
        frc.robot.subsystems.vision.VisionConstants.SINGLE_TAG_MAXIMUM > 0,
        "Single tag maximum distance should be positive");
    assertTrue(
        frc.robot.subsystems.vision.VisionConstants.MULTI_TAG_MAXIMUM > 0,
        "Multi-tag maximum distance should be positive");
  }

  @Test
  @DisplayName("Multi-tag maximum should be >= single-tag maximum")
  void testVisionDistanceHierarchy() {
    assertTrue(
        frc.robot.subsystems.vision.VisionConstants.MULTI_TAG_MAXIMUM >=
            frc.robot.subsystems.vision.VisionConstants.SINGLE_TAG_MAXIMUM,
        "Multi-tag maximum should be >= single-tag maximum");
  }
}

