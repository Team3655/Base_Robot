package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.VisionConstants.PoseObservation;
import frc.robot.subsystems.vision.VisionConstants.TargetObservation;

/**
 * Mock implementation of VisionIO for unit testing.
 * 
 * <p>This allows testing VisionSubsystem without requiring actual cameras.
 * You can set vision observations and verify filtering behavior programmatically.
 */
public class MockVisionIO implements VisionIO {
  private VisionIOInputs inputs = new VisionIOInputs();
  private String cameraName;

  public MockVisionIO(String cameraName) {
    this.cameraName = cameraName;
    inputs.name = cameraName;
    inputs.connected = true;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.name = cameraName;
    inputs.connected = this.inputs.connected;
    inputs.latestObservation = this.inputs.latestObservation;
    inputs.poseObservations = this.inputs.poseObservations.clone();
    inputs.tagIds = this.inputs.tagIds.clone();
  }

  // Test helper methods
  public void setConnected(boolean connected) {
    inputs.connected = connected;
  }

  public void setLatestObservation(TargetObservation observation) {
    inputs.latestObservation = observation;
  }

  public void setPoseObservations(PoseObservation[] observations) {
    inputs.poseObservations = observations.clone();
  }

  public void setTagIds(int[] tagIds) {
    inputs.tagIds = tagIds.clone();
  }
}

