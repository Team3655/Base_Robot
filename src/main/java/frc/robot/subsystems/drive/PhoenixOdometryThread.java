package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import org.littletonrobotics.junction.Logger;

/**
 * Provides an interface for asynchronously reading high-frequency measurements
 * to a set of queues.
 *
 * <p>
 * This version is intended for Phoenix 6 devices on both the RIO and CANivore
 * buses. When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more
 * consistent sampling.
 * This also allows Phoenix Pro users to benefit from lower latency between
 * devices using CANivore
 * time synchronization.
 */
public class PhoenixOdometryThread extends Thread {
  private final List<BaseStatusSignal> signals = new ArrayList<>();
  private final List<Queue<Double>> queues = new ArrayList<>();
  private final Queue<Double> timestampQueue = new ArrayBlockingQueue<>(20);
  private boolean isCANFD = false;
  private boolean started = false;

  private static PhoenixOdometryThread instance = null;

  public static PhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PhoenixOdometryThread();
    }
    return instance;
  }

  private PhoenixOdometryThread() {
    setName("PhoenixOdometryThread");
    setDaemon(true);
  }

  /**
   * Registers a signal for high-frequency odometry sampling
   * Must be called before start().
   */
  public synchronized Queue<Double> registerSignal(ParentDevice device, StatusSignal<Angle> signal) {
    if (started) {
      throw new IllegalStateException("Cannot register signals after thread has started");
    }
    isCANFD = Constants.CANIVORE.isNetworkFD();
    signals.add(signal);
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    queues.add(queue);
    return queue;
  }

  /**
   * Returns the shared timestamp queue. All modules use the same timestamps
   * since signals are sampled together.
   */
  public Queue<Double> getTimestampQueue() {
    return timestampQueue;
  }

  @Override
  public synchronized void start() {
    if (!started && !signals.isEmpty()) {
      started = true;
      super.start();
    }
  }

  @Override
  public void run() {
    BaseStatusSignal[] signalsArray = signals.toArray(new BaseStatusSignal[0]);

    while (true) {
      try {
        // Wait for updates from all signals
        if (isCANFD) {
          BaseStatusSignal.waitForAll(2.0 / DriveConstants.ODOMETRY_FREQUENCY, signalsArray);
        } else {
          // "waitForAll" does not support blocking on multiple
          // signals with a bus that is not CAN FD, regardless
          // of Pro licensing. No reasoning for this behavior
          // is provided by the documentation.
          Thread.sleep((long) (1000.0 / DriveConstants.ODOMETRY_FREQUENCY));
          if (signalsArray.length > 0) {
            BaseStatusSignal.refreshAll(signalsArray);
          }
        }

        // Calculate timestamp with latency compensation
        double timestamp = Logger.getTimestamp() / 1e6;
        double totalLatency = 0.0;
        for (BaseStatusSignal signal : signalsArray) {
          totalLatency += signal.getTimestamp().getLatency();
        }
        if (signalsArray.length > 0) {
          timestamp -= totalLatency / signalsArray.length;
        }

        // Save data to queues (only need odometryLock for queue access)
        DriveSubsystem.odometryLock.lock();
        try {
          // Add timestamp to shared queue
          timestampQueue.offer(timestamp);

          // Add signal values to their respective queues
          for (int i = 0; i < signalsArray.length; i++) {
            queues.get(i).offer(signalsArray[i].getValueAsDouble());
          }
        } finally {
          DriveSubsystem.odometryLock.unlock();
        }
      } catch (InterruptedException e) {
        e.printStackTrace();
        break;
      }
    }
  }
}
