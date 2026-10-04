package first.robot.mechanism.drive;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;

import java.util.Queue;

import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Angle;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import first.lib.util.PhoenixUtil;

public class GyroIOPigeon2 implements GyroIO {
  private static final double kConnectedDebounceSeconds = 0.1;

  private final Pigeon2 m_pigeon2;
  private final StatusSignal<Angle> m_yawSignal;
  private final StatusSignal<Angle> m_pitchSignal;
  private final StatusSignal<Angle> m_rollSignal;

  private final Queue<Double> m_yawQueue;

  private final Debouncer m_connectedDebouncer = new Debouncer(kConnectedDebounceSeconds,
      DebounceType.FALLING);

  public GyroIOPigeon2(Pigeon2 pigeon2) {
    m_pigeon2 = pigeon2;

    PhoenixUtil.tryUntilOk(() -> m_pigeon2.getConfigurator().apply(new Pigeon2Configuration()));
    PhoenixUtil.tryUntilOk(() -> m_pigeon2.getConfigurator().setYaw(Degrees.of(0)));

    m_yawSignal = m_pigeon2.getYaw();
    m_pitchSignal = m_pigeon2.getPitch();
    m_rollSignal = m_pigeon2.getRoll();

    // Yaw is what odometry integrates, so it is sampled as fast as the wheels are. Tilt only scales
    // how far those wheels are believed to have carried the robot, which changes far more slowly.
    BaseStatusSignal.setUpdateFrequencyForAll(PhoenixOdometryThread.getFrequency(), m_yawSignal);
    BaseStatusSignal.setUpdateFrequencyForAll(Hertz.of(50), m_pitchSignal, m_rollSignal);
    m_pigeon2.optimizeBusUtilization();

    // The thread gets its own copy of the signal so that refreshing it here cannot race with the
    // thread's refresh of the same object.
    m_yawQueue = PhoenixOdometryThread.getInstance().registerSignal(m_yawSignal.clone());
  }

  public GyroIOPigeon2(int port, CANBus bus) {
    this(new Pigeon2(port, bus));
  }

  @Override
  public void updateInputs(GyroIOInputsAutoLogged inputs) {
    var status = BaseStatusSignal.refreshAll(m_yawSignal, m_pitchSignal, m_rollSignal);

    // Debounced so that a single dropped frame does not hand odometry over to the wheels and back.
    inputs.connected = m_connectedDebouncer.calculate(status.isOK());
    inputs.yaw = m_yawSignal.getValue();
    inputs.pitch = m_pitchSignal.getValue();
    inputs.roll = m_rollSignal.getValue();

    inputs.odometryYawPositions = m_yawQueue.stream()
        .map(Rotation2d::fromDegrees)
        .toArray(Rotation2d[]::new);
    m_yawQueue.clear();
  }
}
