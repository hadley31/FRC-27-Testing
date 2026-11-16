package first.lib.mechanism;

import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.wpilib.units.Measure;

/**
 * The IO of a mechanism whose measurement means a place rather than a rate, and which therefore has
 * a zero that can be lost: an angle or a position, not a velocity.
 *
 * @param <I> the inputs class this IO populates
 * @param <U> the measure this mechanism's position is expressed in
 */
public interface ZeroableMechanismIO<I extends LoggableInputs, U extends Measure<?>>
    extends G3MechanismIO<I> {
  /**
   * Takes the mechanism's current raw reading as its new zero, and answers with the offset that does
   * it so the mechanism can store it.
   *
   * <p>The IO works this out rather than being told, because it is the only thing that knows the raw
   * reading -- and because the alternative, adding the mechanism's reported position to the offset
   * last stored, reads a value that refreshes once a loop and so would be wrong for any zero landing
   * in the same cycle as the one before it.
   *
   * @return the raw reading just taken as zero, in the same terms {@link #setZeroOffset} expects
   */
  public U zero();

  /**
   * Sets the raw sensor reading that counts as zero, so that the measurements this IO reports and
   * the targets it is given are both in the zeroed frame:
   *
   * <pre>
   * reported = raw - offset
   * commanded = target + offset
   * </pre>
   *
   * <p>Applying the zero is the IO's job and only the IO's, which is what keeps a raw sensor reading
   * from ever leaving it. It is also what lets an IO change <em>how</em> -- subtracting a field,
   * writing a CANcoder magnet offset, seeding an encoder -- without anything above it moving.
   *
   * <p>This is the way a stored offset gets back in: {@code ZeroableMechanism} calls it when the
   * mechanism is built, so a zero found yesterday is in force before the first loop.
   *
   * @param offset the raw reading the mechanism gives when it is physically at zero
   */
  public void setZeroOffset(U offset);
}
