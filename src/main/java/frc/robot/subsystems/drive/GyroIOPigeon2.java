package frc.robot.subsystems.drive;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {

  private CANBus canbus = new CANBus("ctre");
  /**
   * <a href=
   * "https://assets.pokemon.com/assets/cms2/img/pokedex/full//016.png">pidgey</a>
   */
  private final Pigeon2 pidgey = new Pigeon2(20, Constants.CANIVORE);

  private final StatusSignal<Angle> yaw = pidgey.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pidgey.getAngularVelocityZWorld();

  private final Notification gyroConnectedNotification = new Notification(
      NotificationLevel.ERROR,
      "Gyro Disconnected",
      "CTRE Pigeon 2 did not initialize correctly");

  public GyroIOPigeon2() {
    canbus.isNetworkFD();
    pidgey.getConfigurator().apply(new Pigeon2Configuration());
    pidgey.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(DriveConstants.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(100.0);
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pidgey, pidgey.getYaw());

    if (!pidgey.isConnected()) {
      Elastic.sendNotification(gyroConnectedNotification);
    }
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);

    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    inputs.odometryYawPositions = yawPositionQueue.stream()
        .map((Double value) -> Rotation2d.fromDegrees(value))
        .toArray(Rotation2d[]::new);
    yawPositionQueue.clear();
  }

  @Override
  public void resetGyro() {
    pidgey.reset();
  }
}
