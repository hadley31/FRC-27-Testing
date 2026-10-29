package first.lib.auto;

import java.util.Optional;
import java.util.function.Function;

import org.wpilib.command3.Command;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

/**
 * One piece of an auto that the drive team can pick from the dashboard, and that a
 * {@link SegmentedAutoChooser} strings together with others into a single {@link AutoRoutine}.
 *
 * <p>A segment is a recipe rather than a built routine. {@link #build(AutoRoutine)} runs once per
 * occurrence, so the same segment can sit in more than one slot and each occurrence gets its own
 * {@link AutoTrajectory}, with its own progress and its own triggers. That is the whole reason the
 * builder is handed the routine it is being added to instead of closing over one: a segment that
 * captured a trajectory would share that trajectory's state between the two passes the drive team
 * asked for, and the second pass would start out already finished.
 *
 * @param name    what the dashboard shows and what the slots select by. Must be unique within a
 *                chooser.
 * @param builder adds one occurrence of this segment to a routine and reports what it contributed.
 */
public record AutoSegment(String name, Function<AutoRoutine, Run> builder) {
  /**
   * What one occurrence of a segment contributed to the routine it was added to.
   *
   * <p>The paths are what let a chooser check that consecutive segments meet, and let the first
   * segment that drives say where the robot was placed. A segment that does not drive -- a shot, a
   * wait -- has neither, and leaves the seam where the previous driving segment left it.
   *
   * @param command   the command that carries the segment out, awaited in slot order
   * @param entryPath the path this segment starts by driving, if it drives at all
   * @param exitPath  the path this segment finishes by driving, if it drives at all
   */
  public record Run(
      Command command, Optional<AutoTrajectory> entryPath, Optional<AutoTrajectory> exitPath) {
    /**
     * A segment that drives one path.
     *
     * @param path    the path it drives
     * @param command the command that drives it, and anything else the segment does along the way
     * @return the contribution to report
     */
    public static Run driving(AutoTrajectory path, Command command) {
      return new Run(command, Optional.of(path), Optional.of(path));
    }

    /**
     * A segment that drives more than one path.
     *
     * @param entryPath the first path it drives, which is where the segment has to be entered from
     * @param exitPath  the last path it drives, which is where it leaves the robot
     * @param command   the command that drives them, and anything else the segment does along the
     *                  way
     * @return the contribution to report
     */
    public static Run driving(AutoTrajectory entryPath, AutoTrajectory exitPath, Command command) {
      return new Run(command, Optional.of(entryPath), Optional.of(exitPath));
    }

    /**
     * A segment that does not move the chassis, and so can be dropped in anywhere.
     *
     * @param command the command that carries it out
     * @return the contribution to report
     */
    public static Run stationary(Command command) {
      return new Run(command, Optional.empty(), Optional.empty());
    }
  }

  /**
   * Adds a fresh occurrence of this segment to a routine.
   *
   * @param routine the routine being assembled
   * @return what this occurrence contributed
   */
  public Run build(AutoRoutine routine) {
    return builder.apply(routine);
  }
}
