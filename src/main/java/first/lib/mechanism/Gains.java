package first.lib.mechanism;

/**
 * One controller's feedforward and feedback gains, as plain numbers.
 *
 * <p>Deliberately a value type with no connection to how the numbers were obtained. {@code setGains}
 * on an IO implementation takes one of these rather than a tunable, so the hardware layer neither
 * knows nor cares whether a gain came from a constant, a dashboard, or a test -- which is the same
 * convention {@code first.robot.util.Tuning} states for algorithms, and what makes a
 * {@code setGains} implementation testable without NetworkTables.
 *
 * @param kS static friction feedforward
 * @param kV velocity feedforward
 * @param kA acceleration feedforward
 * @param kG gravity feedforward
 * @param kP proportional gain
 * @param kI integral gain
 * @param kD derivative gain
 */
public record Gains(
    double kS, double kV, double kA, double kG, double kP, double kI, double kD) {
  /** All zero: no feedforward and no feedback, which is where an untuned mechanism starts. */
  public static final Gains kZero = new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}
