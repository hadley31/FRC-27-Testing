package first.robot.mechanism.drive;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Newtons;
import static org.wpilib.units.Units.Volts;

import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Force;
import org.wpilib.units.measure.Voltage;

/**
 * Turns a force wanted at the wheel's contact patch into the motor command that produces it.
 *
 * <p>Three conversions, each with a trap in it, which is why they live here rather than inline in
 * each {@link SwerveModuleIO} that needs them. The wheel turns force into torque by its radius. The
 * gearbox multiplies torque, so the motor supplies the wheel's torque <em>divided</em> by the
 * reduction -- the opposite of what the same ratio does to velocity, which is the trap: the two
 * conversions read almost identically and one of them is a division. And a motor's torque is its
 * current times kT, so dividing by kT is what finally turns mechanics into electricity.
 *
 * <p>Getting any of the three backwards produces a feedforward that is wrong by a constant factor,
 * which is the worst kind of wrong: a uniformly scaled feedforward looks exactly like a slightly
 * mistuned one, the velocity controller quietly absorbs the difference, and nothing ever points at
 * the arithmetic. Hence {@code DriveForceFeedforwardTest}, which pins the direction of each
 * conversion rather than only its result.
 */
class DriveForceFeedforward {
  private final double m_wheelRadiusMeters;
  private final double m_gearRatio;
  private final DCMotor m_motor;

  /**
   * @param wheelRadius the rolling radius of the wheel, which is the lever the traction force acts on
   * @param gearRatio   motor rotations per wheel rotation, so greater than one for a reduction
   * @param motor       the motor actually fitted, for its {@code Kt} and {@code R}. Nothing checks
   *                    that the model matches the hardware, and a wrong one is silently wrong.
   */
  DriveForceFeedforward(Distance wheelRadius, double gearRatio, DCMotor motor) {
    m_wheelRadiusMeters = wheelRadius.in(Meters);
    m_gearRatio = gearRatio;
    m_motor = motor;
  }

  /**
   * The current that makes the contact patch push with {@code tractionForce}.
   *
   * <p>This is what a torque current request wants, and it is unaffected by a drive motor whose
   * {@code Feedback.SensorToMechanismRatio} is set to the reduction. That makes closed-loop position
   * and velocity read in wheel rotations, but a feedforward is injected downstream of the conversion
   * and is always in the motor's own terms.
   */
  Current forMotorCurrent(Force tractionForce) {
    double wheelTorqueNm = tractionForce.in(Newtons) * m_wheelRadiusMeters;
    double motorTorqueNm = wheelTorqueNm / m_gearRatio;

    return Amps.of(motorTorqueNm / m_motor.Kt);
  }

  /**
   * The voltage that makes the contact patch push with {@code tractionForce}, for a controller
   * taking volts rather than amps.
   *
   * <p>Only the resistive term. Pushing a current through the winding costs {@code I * R}; the rest
   * of a motor's voltage goes into overcoming back-EMF, which is a function of speed and therefore
   * already the velocity controller's kV term. Adding it here would be charging for it twice.
   */
  Voltage forMotorVoltage(Force tractionForce) {
    return Volts.of(forMotorCurrent(tractionForce).in(Amps) * m_motor.R);
  }
}
