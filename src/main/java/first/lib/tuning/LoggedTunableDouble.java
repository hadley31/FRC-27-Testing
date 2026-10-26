package first.lib.tuning;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.LogTable;
import org.wpilib.networktables.DoubleEntry;
import org.wpilib.networktables.NetworkTableInstance;

/**
 * A tuned number that survives a reboot. See {@link LoggedTunable} for what that buys and what it
 * costs.
 *
 * <p>For a physical quantity -- an angle, a distance, a velocity -- prefer
 * {@link LoggedTunableMeasure}, which keeps the unit attached to the number rather than in a comment
 * next to it. This one is for the genuinely dimensionless: gains, ratios, scale factors.
 */
public final class LoggedTunableDouble extends LoggedTunable<Double> implements DoubleSupplier {
  /**
   * A tuned number that resets to its default every boot.
   *
   * @param path absolute NT path, e.g. {@code "/Tuning/Flywheel/SpinupFactor"}
   * @param codeDefault the value a robot with no dashboard should use
   */
  public static LoggedTunableDouble of(String path, double codeDefault) {
    return new LoggedTunableDouble(path, codeDefault, false, true);
  }

  /**
   * A tuned number the robot saves and restores at boot. See {@link LoggedTunable} on which to
   * reach for: measured numbers persist, behaviour switches do not.
   *
   * @param path absolute NT path
   * @param codeDefault the value used until something is stored on the robot
   */
  public static LoggedTunableDouble persistent(String path, double codeDefault) {
    return new LoggedTunableDouble(path, codeDefault, true, true);
  }

  /**
   * A tuned number belonging to a larger tunable, whose owner calls {@code periodic()}.
   *
   * <p>For values that only make sense together -- the gains of one controller, say -- where the
   * owner has to see a consistent set within one cycle. See
   * {@link TunableDoubleArray#owned(String, double[], boolean)} for the full argument.
   *
   * @param path absolute NT path
   * @param codeDefault the value used until something is stored on the robot
   * @param persistent whether the robot saves the value across a reboot
   */
  public static LoggedTunableDouble owned(String path, double codeDefault, boolean persistent) {
    return new LoggedTunableDouble(path, codeDefault, persistent, false);
  }

  private LoggedTunableDouble(
      String path, double codeDefault, boolean persistent, boolean register) {
    super(path, codeDefault, new Codec(), persistent, register);
  }

  @Override
  public double getAsDouble() {
    return get();
  }

  private static final class Codec implements LoggedTunableCodec<Double> {
    private DoubleEntry m_entry;

    @Override
    public void bind(String path, Double codeDefault, boolean persistent) {
      var topic = NetworkTableInstance.getDefault().getDoubleTopic(path);
      m_entry = topic.getEntry(codeDefault);
      m_entry.setDefault(codeDefault);

      if (persistent) {
        topic.setPersistent(true);
      }
    }

    @Override
    public Double read(Double codeDefault) {
      return m_entry.get(codeDefault);
    }

    @Override
    public void write(Double value) {
      m_entry.set(value);
    }

    @Override
    public void toLog(LogTable table, String key, Double value) {
      table.put(key, (double) value);
    }

    @Override
    public Double fromLog(LogTable table, String key, Double codeDefault) {
      return table.get(key, (double) codeDefault);
    }
  }
}
