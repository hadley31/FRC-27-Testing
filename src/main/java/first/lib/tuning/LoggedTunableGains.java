package first.lib.tuning;

import first.lib.mechanism.Gains;

/**
 * One controller's seven gains, each tuned from a dashboard and each surviving a reboot.
 *
 * <p>The persistence is the point. Gains used to be published through {@code org.wpilib.tunable},
 * which stores nothing: a kP found at 4pm was gone after the next code deploy, so tuning meant
 * writing numbers on paper and typing them back in. These are persistent NT topics, so a session
 * survives the restarts between matches -- and {@link LoggedTunables}' alert says, from the driver
 * station, that the robot is running gains the source does not contain.
 *
 * <p>Paths are nested under the prefix, so a mechanism named {@code Turret} gets
 * {@code /Tuning/Turret/kP} and friends.
 *
 * <p>The seven are refreshed together by {@link #periodic()} rather than individually by
 * AdvantageKit, so a consumer always sees one coherent set of gains rather than a mix of two
 * cycles' worth. {@code TunableDoubleArray.owned} documents the same argument at greater length.
 */
public final class LoggedTunableGains {
  private final LoggedTunableDouble m_kS;
  private final LoggedTunableDouble m_kV;
  private final LoggedTunableDouble m_kA;
  private final LoggedTunableDouble m_kG;
  private final LoggedTunableDouble m_kP;
  private final LoggedTunableDouble m_kI;
  private final LoggedTunableDouble m_kD;

  /**
   * @param pathPrefix the absolute NT path prefix, e.g. {@code "/Tuning/Turret"}
   * @param codeDefaults the gains committed in code, used until something is stored on the robot
   */
  public LoggedTunableGains(String pathPrefix, Gains codeDefaults) {
    m_kS = LoggedTunableDouble.owned(pathPrefix + "/kS", codeDefaults.kS(), true);
    m_kV = LoggedTunableDouble.owned(pathPrefix + "/kV", codeDefaults.kV(), true);
    m_kA = LoggedTunableDouble.owned(pathPrefix + "/kA", codeDefaults.kA(), true);
    m_kG = LoggedTunableDouble.owned(pathPrefix + "/kG", codeDefaults.kG(), true);
    m_kP = LoggedTunableDouble.owned(pathPrefix + "/kP", codeDefaults.kP(), true);
    m_kI = LoggedTunableDouble.owned(pathPrefix + "/kI", codeDefaults.kI(), true);
    m_kD = LoggedTunableDouble.owned(pathPrefix + "/kD", codeDefaults.kD(), true);
  }

  /** An untuned bundle, all gains zero. */
  public LoggedTunableGains(String pathPrefix) {
    this(pathPrefix, Gains.kZero);
  }

  /**
   * The current gains, as plain numbers for an IO implementation to apply.
   *
   * <p>Built fresh rather than cached: it is seven field reads into a record, and a cache here would
   * only add a way for the returned gains to disagree with the ones a dashboard is showing.
   */
  public Gains gains() {
    return new Gains(
        m_kS.getAsDouble(),
        m_kV.getAsDouble(),
        m_kA.getAsDouble(),
        m_kG.getAsDouble(),
        m_kP.getAsDouble(),
        m_kI.getAsDouble(),
        m_kD.getAsDouble());
  }

  /**
   * Whether any gain changed during this loop's refresh.
   *
   * <p>See {@link LoggedTunable#changedThisLoop()} for why this is a one-loop query rather than a
   * latch, and why a consumer must also apply the gains once when it starts.
   */
  public boolean changedThisLoop() {
    return m_kS.changedThisLoop()
        || m_kV.changedThisLoop()
        || m_kA.changedThisLoop()
        || m_kG.changedThisLoop()
        || m_kP.changedThisLoop()
        || m_kI.changedThisLoop()
        || m_kD.changedThisLoop();
  }

  /**
   * Refreshes all seven from NetworkTables, or from the log during replay.
   *
   * <p>Call once per loop, before anything reads the gains. {@code G3Mechanism} arranges this
   * through {@code Scheduler.addPeriodic}, whose callbacks run before any command in the same
   * {@code Scheduler.run()}.
   */
  public void periodic() {
    m_kS.periodic();
    m_kV.periodic();
    m_kA.periodic();
    m_kG.periodic();
    m_kP.periodic();
    m_kI.periodic();
    m_kD.periodic();
  }
}
