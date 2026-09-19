package first.robot.mechanism.led;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import org.wpilib.command3.Mechanism;
import org.wpilib.command3.NeedsNameBuilderStage;
import org.wpilib.hardware.led.AddressableLED;
import org.wpilib.hardware.led.AddressableLEDBuffer;
import org.wpilib.hardware.led.AddressableLEDBufferView;
import org.wpilib.hardware.led.LEDPattern;
import org.wpilib.simulation.AddressableLEDSim;

import first.lib.command3.RangedLEDCommand;

/**
 * LED mechanism representing a single addressable LED strip. Unlike most mechanisms, an LED strip
 * can be safely split between commands: two commands that each own a disjoint range of pixels may
 * run at the same time, while two commands whose ranges overlap may not. This uses a single {@link
 * Mechanism} requirement (this {@code LED} instance itself), so bookkeeping like {@link
 * org.wpilib.command3.Scheduler#getRunningCommandsFor}, default commands, and disabled-mode safety
 * all behave normally. Range-overlap exclusivity itself is handled by {@link RangedLEDCommand}.
 *
 * <p>{@link Segment#runOverlay(LEDPattern, int)} offers a second, non-exclusive way to paint: any
 * number of overlays may cover the same pixels at once, with priority resolved each tick by a
 * compositing pass instead of by preempting commands.
 */
public class LED implements Mechanism {
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;
  private final AddressableLEDSim m_sim;
  private final int m_length;

  /**
   * Overlay paints queued by currently-running overlay commands during the current scheduler tick.
   * Consumed and cleared by the compositing pass below on the following tick; nothing needs to be
   * unregistered when an overlay command stops; it simply stops appending here.
   */
  private final List<OverlayPaint> m_overlayPaints = new ArrayList<>();

  private record OverlayPaint(AddressableLEDBufferView view, int priority, LEDPattern pattern) {
  }

  public LED(int port, int length) {
    m_length = length;
    m_buffer = new AddressableLEDBuffer(length);
    m_led = new AddressableLED(port);
    m_led.setLength(length);
    m_led.setData(m_buffer);
    m_sim = new AddressableLEDSim(m_led);

    // Once per scheduler tick, after every command has had a chance to write into the buffer (or
    // queue an overlay paint): composite queued overlays on top of the buffer in priority order
    // (lowest first, so the highest-priority overlay paints last and ends up on top), push the
    // result to the physical strip, then clear the queue so only this tick's overlays are used
    // next time.
    getRegisteredScheduler().addPeriodic(() -> {
      m_overlayPaints.stream()
          .sorted(Comparator.comparingInt(OverlayPaint::priority).reversed())
          .forEach(paint -> paint.pattern().applyTo(paint.view()));
      m_led.setData(m_buffer);
      m_overlayPaints.clear();
    });
  }

  /**
   * Gets a handle representing an inclusive range of LEDs, {@code [startIndex, endIndex]}.
   * Commands built from the returned {@link Segment} conflict with each other only if their ranges
   * overlap; non-overlapping segments (e.g. {@code range(0, 49)} and {@code range(50, 99)}) can run
   * concurrently.
   *
   * @param startIndex the first LED index in the range, inclusive
   * @param endIndex the last LED index in the range, inclusive
   */
  public Segment range(int startIndex, int endIndex) {
    if (startIndex < 0 || endIndex >= m_length || startIndex > endIndex) {
      throw new IllegalArgumentException(
          "Invalid range [%d, %d] for a strip of length %d".formatted(startIndex, endIndex, m_length));
    }
    return new Segment(startIndex, endIndex);
  }

  /** Gets a handle representing the entire LED strip. Equivalent to {@code range(0, length - 1)}. */
  public Segment full() {
    return range(0, m_length - 1);
  }

  /**
   * Gets the simulated LED device backing this strip. Real {@link AddressableLED#setData} calls
   * are already mirrored into simulation automatically at the HAL layer, so this isn't needed for
   * writing; it's useful in unit tests to confirm data actually reached the (simulated) hardware,
   * as opposed to only checking values already held locally, which would just prove the code agrees
   * with itself regardless of whether anything was actually sent.
   */
  public AddressableLEDSim getSim() {
    return m_sim;
  }

  /** A handle for building commands scoped to a contiguous range of LEDs. */
  public final class Segment {
    private final int m_start;
    private final int m_end;
    private final AddressableLEDBufferView m_view;

    private Segment(int startIndex, int endIndex) {
      m_start = startIndex;
      m_end = endIndex;
      m_view = m_buffer.createView(startIndex, endIndex);
    }

    /**
     * Builds a command that repeatedly applies {@code pattern} to this range only, leaving the
     * rest of the strip's buffer untouched. Useful for animated patterns, which must be
     * re-applied periodically to advance. Conflicts with any other command (overlay or not) whose
     * range overlaps this one.
     */
    public NeedsNameBuilderStage runPattern(LEDPattern pattern) {
      return RangedLEDCommand.wrapping(
          LED.this.run(coroutine -> {
            while (true) {
              pattern.applyTo(m_view);
              coroutine.yield();
            }
          }),
          m_start,
          m_end,
          false);
    }

    /**
     * Builds a command that queues {@code pattern} to be composited on top of this range every
     * tick, at the given priority, instead of exclusively owning it. Never conflicts with another
     * command over range overlap; when two overlays (or an overlay and a {@link
     * #runPattern(LEDPattern) base command}) cover the same pixels, the highest-priority one wins
     * those pixels for that tick. If nothing else is painting underneath, whatever was last written
     * to those pixels persists once this overlay stops.
     */
    public NeedsNameBuilderStage runOverlay(LEDPattern pattern, int priority) {
      return RangedLEDCommand.wrapping(
          LED.this.run(coroutine -> {
            while (true) {
              m_overlayPaints.add(new OverlayPaint(m_view, priority, pattern));
              coroutine.yield();
            }
          }),
          m_start,
          m_end,
          true)
          .withPriority(priority);
    }
  }
}
