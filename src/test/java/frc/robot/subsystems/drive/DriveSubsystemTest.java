package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Unit tests for DriveSubsystem.
 * 
 * <p>
 * These tests verify:
 * <ul>
 * <li>Kinematics calculations (chassis speeds to module states)</li>
 * <li>Module state management</li>
 * <li>Stop functionality</li>
 * <li>State reporting</li>
 * </ul>
 * 
 * <p>
 * Note: These tests use mock IO implementations, so no hardware is required.
 */
@DisplayName("DriveSubsystem Tests")
class DriveSubsystemTest {

  private DriveSubsystem drive;
  private MockGyroIO mockGyro;
  private MockModuleIO[] mockModules;

  @BeforeEach
  void setUp() {
    // Create mock IO implementations
    mockGyro = new MockGyroIO();
    mockGyro.setConnected(true);
    mockGyro.setYawPosition(new Rotation2d());

    mockModules = new MockModuleIO[4];
    for (int i = 0; i < 4; i++) {
      mockModules[i] = new MockModuleIO();
    }

    // Create DriveSubsystem with mocks
    drive = new DriveSubsystem(
        mockGyro,
        mockModules[0],
        mockModules[1],
        mockModules[2],
        mockModules[3]);

  }

  @Test
  @DisplayName("DriveSubsystem should be created successfully")
  void testSubsystemCreation() {
    assertNotNull(drive, "DriveSubsystem should be created");
  }

  @Test
  @DisplayName("runVelocity should convert chassis speeds to module states correctly")
  void testRunVelocity() {
    // Test forward motion - all modules should point forward (0 degrees)
    ChassisSpeeds forwardSpeeds = new ChassisSpeeds(1.0, 0.0, 0.0); // 1 m/s forward
    drive.runVelocity(forwardSpeeds);

    // For forward motion, all modules should point forward (0 degrees)
    // and have forward velocity
    for (MockModuleIO module : mockModules) {
      // Verify turn setpoint is forward (0 radians)
      Rotation2d turnSetpoint = module.getTurnSetpoint();
      assertEquals(0.0, turnSetpoint.getRadians(), 0.01,
          "All modules should point forward (0 degrees) for forward motion");

      // Verify drive velocity is set and positive (forward)
      double driveVelocity = module.getDriveVelocity();
      assertTrue(driveVelocity > 0.0,
          String.format("Module should have positive forward velocity, got %.2f",
              driveVelocity));

      // Verify velocity is reasonable (should convert to ~1 m/s)
      double speedMps = driveVelocity * DriveConstants.WHEEL_RADIUS;
      assertTrue(speedMps > 0.5 && speedMps < 2.0,
          String.format("Module speed should be around 1 m/s, got %.2f m/s", speedMps));
    }
  }

  @Test
  @DisplayName("stop should set all modules to zero velocity")
  void testStop() {
    // First, set modules to some motion state
    drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0));

    // Verify modules received setpoints (they should point forward)
    for (MockModuleIO module : mockModules) {
      Rotation2d setpoint = module.getTurnSetpoint();
      assertNotNull(setpoint, "Module should have a turn setpoint after runVelocity");
    }

    // Then stop - this sets all voltages to zero
    drive.stop();

    // Verify all modules have zero voltage (stopped)
    for (int i = 0; i < mockModules.length; i++) {
      MockModuleIO module = mockModules[i];
      assertEquals(0.0, module.getDriveVoltage(), 0.001,
          String.format("Module %d drive voltage should be 0 after stop", i));
      assertEquals(0.0, module.getTurnVoltage(), 0.001,
          String.format("Module %d turn voltage should be 0 after stop", i));
    }
  }

  @Test
  @DisplayName("Subsystem should handle periodic updates")
  void testPeriodic() {
    // Test that periodic doesn't crash with default state
    drive.periodic();

    // Periodic should complete without errors
    assertNotNull(drive, "DriveSubsystem should still exist after periodic");
  }

  @Test
  @DisplayName("stopWithX should orient modules in X pattern")
  void testStopWithX() {
    // Set modules to their expected X-pattern angles before calling stopWithX
    // This simulates that the modules are already at those positions, which is what
    // resetHeadings expects. In the real robot, the modules are already at their
    // current positions when resetHeadings is called.
    for (int i = 0; i < mockModules.length; i++) {
      Rotation2d expectedAngle = DriveConstants.moduleTranslations[i].getAngle();
      mockModules[i].setTurnPositionInput(expectedAngle);
    }

    // Call periodic to update module inputs before stopWithX
    drive.periodic();

    // Now test stopWithX - modules should be oriented at 45-degree angles
    drive.stopWithX();

    // Verify modules are in X pattern - each module should point at its corner
    // angle
    // Module translations are at: FL (45°), FR (-45°), BL (135°), BR (-135°)
    double tolerance = 0.03;

    for (int i = 0; i < mockModules.length; i++) {
      Rotation2d expectedAngle = DriveConstants.moduleTranslations[i].getAngle();
      Rotation2d actualAngle = mockModules[i].getTurnSetpoint();

      // Check if angles match
      double expectedRad = expectedAngle.getRadians();
      double actualRad = actualAngle.getRadians();

      // Normalize to [-pi, pi] range
      while (expectedRad > Math.PI)
        expectedRad -= 2 * Math.PI;
      while (expectedRad < -Math.PI)
        expectedRad += 2 * Math.PI;

      while (actualRad > Math.PI)
        actualRad -= 2 * Math.PI;
      while (actualRad < -Math.PI)
        actualRad += 2 * Math.PI;

      assertEquals(expectedRad, actualRad, tolerance,
          String.format("Module %d should point at angle %.2f radians, got %.2f",
              i, expectedRad, actualRad));

      // Also verify modules are stopped
      assertEquals(0.0, mockModules[i].getDriveVoltage(), 0.001,
          String.format("Module %d drive voltage should be 0", i));
      assertEquals(0.0, mockModules[i].getTurnVoltage(), 0.001,
          String.format("Module %d turn voltage should be 0", i));
    }
  }

  @Test
  @DisplayName("Kinematics should desaturate wheel speeds correctly")
  void testKinematicsDesaturation() {
    // Create a chassis speed that would exceed max wheel speed
    double maxSpeed = DriveConstants.MAX_LINEAR_SPEED;
    ChassisSpeeds excessiveSpeeds = new ChassisSpeeds(maxSpeed * 2, maxSpeed * 2, 0.0);

    // Run velocity - should desaturate automatically
    drive.runVelocity(excessiveSpeeds);

    // The kinematics.desaturateWheelSpeeds is called internally
    // After desaturation, module speeds should not exceed MAX_LINEAR_SPEED
    // Convert module velocities from rotations/sec to m/s for verification
    double maxModuleSpeedMps = DriveConstants.MAX_LINEAR_SPEED;
    double tolerance = 0.1; // Allow small tolerance

    for (int i = 0; i < mockModules.length; i++) {
      MockModuleIO module = mockModules[i];
      double moduleVelocityRotPerSec = module.getDriveVelocity();
      double moduleSpeedMps = moduleVelocityRotPerSec * DriveConstants.WHEEL_RADIUS;

      // Desaturated speeds should not exceed maximum
      assertTrue(Math.abs(moduleSpeedMps) <= maxModuleSpeedMps + tolerance,
          String.format("Module %d speed %.2f m/s should not exceed max %.2f m/s",
              i, moduleSpeedMps, maxModuleSpeedMps));
    }
  }

  @Test
  @DisplayName("Module states should be accessible via mocks")
  void testGetModuleStates() {
    // Set up initial state with forward motion
    ChassisSpeeds speeds = new ChassisSpeeds(0.5, 0.0, 0.0);
    drive.runVelocity(speeds);

    // Verify that all modules received setpoints
    // For forward motion at 0.5 m/s, all modules should point forward
    int modulesSet = 0;
    for (MockModuleIO module : mockModules) {
      Rotation2d setpoint = module.getTurnSetpoint();
      // Forward motion means modules point forward (0 radians)
      if (Math.abs(setpoint.getRadians()) < 0.1) {
        modulesSet++;
      }

      // Drive velocity should be set (non-zero for forward motion)
      double velocity = module.getDriveVelocity();
      // Velocity is in rotations/sec, should be positive for forward
      assertTrue(velocity >= 0.0,
          "Drive velocity should be non-negative for forward motion");
    }

    // At least all 4 modules should have received forward setpoints
    assertEquals(4, modulesSet,
        "All 4 modules should point forward for pure forward motion");

    // Verify the subsystem is in a valid state
    assertNotNull(drive, "DriveSubsystem should exist");
  }

  @Test
  @DisplayName("Rotational motion should set modules correctly")
  void testRotationalMotion() {
    // Test pure rotation - modules should be oriented to create rotation
    double angularVelocity = 1.0; // 1 rad/s rotation
    ChassisSpeeds rotationalSpeeds = new ChassisSpeeds(0.0, 0.0, angularVelocity);
    drive.runVelocity(rotationalSpeeds);

    // For pure rotation, modules should be oriented tangentially
    // All modules should have velocity set
    int modulesWithVelocity = 0;
    for (MockModuleIO module : mockModules) {
      double velocity = module.getDriveVelocity();
      if (Math.abs(velocity) > 0.01) {
        modulesWithVelocity++;
      }

      // Turn setpoint should be set (not necessarily zero for rotation)
      Rotation2d setpoint = module.getTurnSetpoint();
      assertNotNull(setpoint, "Module should have a turn setpoint");
    }

    // All modules should participate in rotation
    assertEquals(4, modulesWithVelocity,
        "All 4 modules should have velocity set for rotational motion");
  }
}
