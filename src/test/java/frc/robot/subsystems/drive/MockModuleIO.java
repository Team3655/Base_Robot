package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Mock implementation of ModuleIO for unit testing.
 * 
 * <p>This allows testing DriveSubsystem without requiring actual hardware.
 * You can set inputs and verify outputs programmatically.
 */
public class MockModuleIO implements ModuleIO {
  private ModuleIOInputs inputs = new ModuleIOInputs();
  private double driveVoltage = 0.0;
  private double turnVoltage = 0.0;
  @SuppressWarnings("unused")
  private double driveVelocity = 3;
  private Rotation2d turnSetpoint = new Rotation2d();

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Copy our internal state to inputs
    inputs.drivePositionRad = this.inputs.drivePositionRad;
    inputs.driveVelocityRadPerSec = this.inputs.driveVelocityRadPerSec;
    inputs.driveAppliedVolts = driveVoltage;
    inputs.driveCurrentAmps = this.inputs.driveCurrentAmps;
    inputs.turnAbsolutePosition = this.inputs.turnAbsolutePosition;
    inputs.turnPosition = this.inputs.turnPosition;
    inputs.turnPositionError = this.inputs.turnPositionError;
    inputs.turnVelocityRadPerSec = this.inputs.turnVelocityRadPerSec;
    inputs.turnAppliedVolts = turnVoltage;
    inputs.turnCurrentAmps = this.inputs.turnCurrentAmps;
    inputs.odometryTimestamps = this.inputs.odometryTimestamps.clone();
    inputs.odometryDrivePositionsRad = this.inputs.odometryDrivePositionsRad.clone();
    inputs.odometryTurnPositions = this.inputs.odometryTurnPositions.clone();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveVoltage = volts;
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnVoltage = volts;
  }

  @Override
  public void setTurnPosition(Rotation2d setpoint) {
    turnSetpoint = setpoint;
  }

  @Override
  public void setDriveVelocity(double rotationsPerSecond) {
    driveVelocity = rotationsPerSecond;
  }

  // Test helper methods
  public void setDrivePosition(double positionRad) {
    inputs.drivePositionRad = positionRad;
  }

  public void setDriveVelocityInput(double velocityRadPerSec) {
    inputs.driveVelocityRadPerSec = velocityRadPerSec;
  }

  public void setTurnPositionInput(Rotation2d position) {
    inputs.turnPosition = position;
    inputs.turnAbsolutePosition = position;
  }

  public void setOdometryData(double[] timestamps, double[] drivePositions, Rotation2d[] turnPositions) {
    inputs.odometryTimestamps = timestamps.clone();
    inputs.odometryDrivePositionsRad = drivePositions.clone();
    inputs.odometryTurnPositions = turnPositions.clone();
  }

  public double getDriveVoltage() {
    return driveVoltage;
  }

  public double getTurnVoltage() {
    return turnVoltage;
  }

  public Rotation2d getTurnSetpoint() {
    return turnSetpoint;
  }

  public double getDriveVelocity() {
    return driveVelocity;
  }
}

