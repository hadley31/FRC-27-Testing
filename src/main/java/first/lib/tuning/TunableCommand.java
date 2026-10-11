package first.lib.tuning;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.tunable.ComplexTunable;
import org.wpilib.tunable.TunableConfig;
import org.wpilib.tunable.TunableOption;
import org.wpilib.tunable.TunableTable;

/**
 * A commands v3 {@link Command} published as a dashboard button.
 *
 * <p>Commands v2 made this free: {@code Command} was {@code Sendable}, so {@code
 * SmartDashboard.putData(name, command)} produced a widget that scheduled the command when pressed and
 * cancelled it when pressed again. Commands v3 commands are plain objects with no dashboard
 * integration of their own -- the scheduler as a whole is serializable, but an individual command is
 * not -- so the binding lives here instead.
 *
 * <p>The published shape is the one dashboards already know, and is copied from how commands v2 now
 * publishes itself since Sendable was replaced by the tunable API: a {@code "Command"}-typed table with
 * a read-only {@code name} and a writable {@code running}. That is exactly what Glass looks for, so a
 * command published through this class appears as a run/cancel button rather than a bare checkbox.
 *
 * <p>The command is held for the lifetime of this object. Commands v3 <em>builders</em> are single-use,
 * but a built command is not: it can be scheduled, cancelled and scheduled again indefinitely, which is
 * the whole point of a button.
 */
public final class TunableCommand implements ComplexTunable {
  private final Command m_command;
  private final Scheduler m_scheduler;

  /**
   * Wraps a command for publishing.
   *
   * @param command the command to run when the button is pressed
   * @param scheduler the scheduler to run it on, normally the owning mechanism's {@code
   *     getRegisteredScheduler()}
   */
  public TunableCommand(Command command, Scheduler scheduler) {
    m_command = command;
    m_scheduler = scheduler;
  }

  public TunableCommand(Command command) {
    this(command, Scheduler.getDefault());
  }

  @Override
  public String getTunableType() {
    // Matched verbatim by Glass to pick the command widget over a generic table of values.
    return "Command";
  }

  @Override
  public void publishTunable(TunableTable table) {
    table.publishValue(
        "name",
        m_command::name,
        null,
        String.class,
        TunableConfig.of(TunableOption.IMMUTABLE, TunableOption.GET_ON_CHANGE));

    // ALWAYS_GET so the button stops looking pressed the moment the command ends or is interrupted by
    // a conflicting one, rather than only when something writes to the topic.
    table.publishBoolean(
        "running",
        () -> m_scheduler.isScheduledOrRunning(m_command),
        this::setRunning,
        TunableConfig.of(TunableOption.ALWAYS_GET));
  }

  private void setRunning(boolean running) {
    if (!running) {
      if (m_scheduler.isScheduledOrRunning(m_command)) {
        m_scheduler.cancel(m_command);
      }
      return;
    }

    if (m_scheduler.isScheduledOrRunning(m_command)) {
      return;
    }

    var result = m_scheduler.schedule(m_command);

    if (!result.successful()) {
      // Scheduling fails silently -- a command that loses to a higher-priority one, or that wants a
      // mechanism which is not controllable while disabled, simply does not start. Without this the
      // button just looks broken.
      DriverStationErrors.reportWarning(
          "Could not schedule " + m_command.name() + ": " + result, false);
    }
  }
}
