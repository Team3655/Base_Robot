package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Mock implementation of GyroIO for unit testing.
 * 
 * <p>
 * This allows testing DriveSubsystem without requiring actual hardware.
 * You can set gyro values and verify behavior programmatically.
 */
public class MockGyroIO implements GyroIO {
  private GyroIOInputs inputs = new GyroIOInputs();
  private Rotation2d yawPosition = new Rotation2d();

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = this.inputs.connected;
    inputs.yawPosition = yawPosition;
    inputs.odometryYawPositions = this.inputs.odometryYawPositions.clone();
    inputs.yawVelocityRadPerSec = this.inputs.yawVelocityRadPerSec;
  }

  @Override
  public void resetGyro() {
    yawPosition = new Rotation2d();
  }

  @Override
  public void setRotation(Rotation2d angle) {
    yawPosition = angle;
  }

  // Test helper methods
  public void setYawPosition(Rotation2d yaw) {
    yawPosition = yaw;
  }

  public void setConnected(boolean connected) {
    inputs.connected = connected;
  }

  public void setOdometryYawPositions(Rotation2d[] yawPositions) {
    inputs.odometryYawPositions = yawPositions.clone();
  }

  public void setYawVelocity(double velocityRadPerSec) {
    inputs.yawVelocityRadPerSec = velocityRadPerSec;
  }
}
