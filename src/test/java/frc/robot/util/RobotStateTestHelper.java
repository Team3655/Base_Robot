package frc.robot.util;

import frc.robot.RobotState;

/**
 * Test utility helper for RobotState.
 * 
 * <p>Provides convenient methods for testing vision measurement filtering
 * in VisionSubsystem tests.
 */
public class RobotStateTestHelper {
  
  /**
   * Resets the vision measurement counter before running a test.
   * Call this in @BeforeEach or at the start of each test that checks filtering.
   */
  public static void resetVisionMeasurementCounter() {
    RobotState.resetVisionMeasurementCounter();
  }
  
  /**
   * Gets the number of vision measurements that were added to RobotState.
   * Use this to verify that valid observations were accepted and invalid ones were rejected.
   */
  public static int getVisionMeasurementCount() {
    return RobotState.getVisionMeasurementCount();
  }
  
  /**
   * Verifies that exactly the expected number of vision measurements were added.
   * Useful for testing that observations were filtered correctly.
   * 
   * @param expectedCount The expected number of measurements
   * @param message Error message if count doesn't match
   * @throws AssertionError if count doesn't match
   */
  public static void assertVisionMeasurementCount(int expectedCount, String message) {
    int actualCount = getVisionMeasurementCount();
    if (actualCount != expectedCount) {
      throw new AssertionError(
          String.format("%s: Expected %d vision measurements, but got %d",
              message, expectedCount, actualCount));
    }
  }
}

