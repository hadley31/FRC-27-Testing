package first.robot.mechanism.drive;

import static first.robot.util.Constants.ElectricalConstants.CAN_BUS;
import static org.wpilib.units.Units.Hertz;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.wpilib.system.Timer;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Frequency;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

/**
 * Samples the drivetrain's position signals faster than the main loop runs, so that odometry
 * integrates the path the robot actually took rather than a 50 Hz approximation of it.
 *
 * <p>On a CAN FD bus the thread blocks on {@code waitForAll}, which both paces it off the devices
 * themselves and, with a Phoenix Pro licence, gives every signal in a pass the same timestamp. On a
 * bus that is not CAN FD, Phoenix does not support blocking on several signals at once, so the thread
 * falls back to sleeping and refreshing.
 */
public final class PhoenixOdometryThread extends Thread {
  private static final Frequency kFrequency = CAN_BUS.isNetworkFD() ? Hertz.of(250) : Hertz.of(100);

  /**
   * Held while the main loop copies the queues out, so that a single pass of this thread is never
   * split across two cycles of odometry.
   */
  private static final Lock kLock = new ReentrantLock();

  private static PhoenixOdometryThread s_instance;

  private final Lock m_signalsLock = new ReentrantLock();
  private final List<BaseStatusSignal> m_signals = new ArrayList<>();
  private final List<Queue<Double>> m_queues = new ArrayList<>();
  private final List<Queue<Double>> m_timestampQueues = new ArrayList<>();

  private PhoenixOdometryThread() {
    setName("PhoenixOdometryThread");
    setDaemon(true);
  }

  public static synchronized PhoenixOdometryThread getInstance() {
    if (s_instance == null) {
      s_instance = new PhoenixOdometryThread();
    }
    return s_instance;
  }

  /** How often this thread samples. Signals registered with it should be published at least as fast. */
  public static Frequency getFrequency() {
    return kFrequency;
  }

  /** The lock a reader must hold while draining the queues this thread fills. */
  public static Lock getLock() {
    return kLock;
  }

  /** Starts sampling, unless nothing has registered — which is the case in simulation and replay. */
  @Override
  public void start() {
    if (!m_timestampQueues.isEmpty()) {
      super.start();
    }
  }

  /**
   * Registers a signal to sample. Pass a {@link StatusSignal#clone()} rather than the signal the
   * caller refreshes itself, so the two cannot race.
   *
   * @return the queue this thread will append the signal's values to
   */
  public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);

    m_signalsLock.lock();
    kLock.lock();
    try {
      m_signals.add(signal);
      m_queues.add(queue);
    } finally {
      m_signalsLock.unlock();
      kLock.unlock();
    }

    return queue;
  }

  /** @return a queue this thread will append one timestamp per sampling pass to */
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);

    kLock.lock();
    try {
      m_timestampQueues.add(queue);
    } finally {
      kLock.unlock();
    }

    return queue;
  }

  @Override
  public void run() {
    while (true) {
      m_signalsLock.lock();
      try {
        if (m_signals.isEmpty()) {
          Thread.sleep((long) (1000.0 / kFrequency.in(Hertz)));
        } else if (CAN_BUS.isNetworkFD()) {
          BaseStatusSignal.waitForAll(2.0 / kFrequency.in(Hertz), m_signals);
        } else {
          Thread.sleep((long) (1000.0 / kFrequency.in(Hertz)));
          BaseStatusSignal.refreshAll(m_signals);
        }
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        return;
      } finally {
        m_signalsLock.unlock();
      }

      kLock.lock();
      try {
        // Phoenix's own timestamps are not on the same clock as the rest of the robot code, so the
        // sample is dated by backing the average bus latency out of the current time instead. That
        // is approximate, but it is comparable with the timestamps vision observations carry.
        double totalLatency = 0.0;
        for (BaseStatusSignal signal : m_signals) {
          totalLatency += signal.getTimestamp().getLatency();
        }
        double timestamp = Timer.getTimestamp()
            - (m_signals.isEmpty() ? 0.0 : totalLatency / m_signals.size());

        // A full queue drops the sample rather than blocking, which is the right trade: this thread
        // falling behind the robot would be worse than odometry missing a step.
        for (int i = 0; i < m_signals.size(); i++) {
          m_queues.get(i).offer(m_signals.get(i).getValueAsDouble());
        }
        for (Queue<Double> timestampQueue : m_timestampQueues) {
          timestampQueue.offer(timestamp);
        }
      } finally {
        kLock.unlock();
      }
    }
  }
}
