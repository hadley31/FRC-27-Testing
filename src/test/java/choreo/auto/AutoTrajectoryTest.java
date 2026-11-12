package choreo.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Mechanism;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Pose2d;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

/**
 * Covers the velocities a trajectory reports at its ends, which are what a handoff between two
 * paths is judged by: a path that does not stop where the next one starts moving is the failure
 * these accessors exist to make visible.
 */
class AutoTrajectoryTest {
  private static final double kEpsilon = 1e-9;

  @BeforeAll
  static void initializeHal() {
    // Choreo reports usage when a factory is built, and AdvantageKit's conduit segfaults if the
    // driver station has not been refreshed once.
    assertTrue(HAL.initialize(), "HAL failed to initialize");
    DriverStationBackend.refreshData();
  }

  private static SwerveSample sample(double t, double vx, double vy, double omega) {
    return new SwerveSample(
        t, 0.0, 0.0, 0.0, vx, vy, omega, 0.0, 0.0, 0.0, new double[4], new double[4]);
  }

  private static AutoTrajectory trajectory(SwerveSample... samples) {
    AutoFactory factory = new AutoFactory(
        Pose2d::new,
        pose -> {
        },
        sample -> {
        },
        // Flipping off, so the ends resolve without an alliance to flip them for.
        false,
        new Mechanism() {
        },
        (trajectory, isStart) -> {
        });

    return factory.newRoutine("Test Routine")
        .trajectory(new Trajectory<>("Test", List.of(samples), List.of(), List.of()));
  }

  @Test
  void theFinalVelocitiesAreTheLastSamples() {
    AutoTrajectory path = trajectory(
        sample(0.0, 1.0, 2.0, 3.0),
        sample(1.0, 4.0, 5.0, 6.0));

    var velocities = path.getFinalChassisVelocities().orElseThrow();

    assertEquals(4.0, velocities.vx, kEpsilon);
    assertEquals(5.0, velocities.vy, kEpsilon);
    assertEquals(6.0, velocities.omega, kEpsilon);
  }

  @Test
  void theInitialVelocitiesAreTheFirstSamples() {
    AutoTrajectory path = trajectory(
        sample(0.0, 1.0, 2.0, 3.0),
        sample(1.0, 4.0, 5.0, 6.0));

    var velocities = path.getInitialChassisVelocities().orElseThrow();

    assertEquals(1.0, velocities.vx, kEpsilon);
    assertEquals(2.0, velocities.vy, kEpsilon);
    assertEquals(3.0, velocities.omega, kEpsilon);
  }

  @Test
  void aTrajectoryThatFailedToLoadHasNoVelocities() {
    // What an AutoTrajectory holds when the .traj file was missing: no samples at all.
    AutoTrajectory path = trajectory();

    assertTrue(path.getInitialChassisVelocities().isEmpty());
    assertTrue(path.getFinalChassisVelocities().isEmpty());
  }
}
