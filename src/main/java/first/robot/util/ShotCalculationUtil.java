package first.robot.util;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Feet;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;

import java.util.Optional;
import java.util.function.Supplier;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

import first.lib.InterpolatingMeasureTreeMap;

public class ShotCalculationUtil {
  private static final Time kTimeOfFlightTolerance = Seconds.of(0.05);
  private static final int kMaximumIterations = 100;
  private static final double kLatencyCompensationSeconds = 0.03;

  private Supplier<Transform3d> m_turretTransformSupplier;
  private Supplier<Translation3d> m_targetTranslationSupplier;
  private Supplier<Transform2d> m_robotTransformSupplier;

  private final InterpolatingMeasureTreeMap<Distance, Time> m_scoringTimeOfFlightMap = InterpolatingMeasureTreeMap
      .createDistanceToTimeMap()
      .put(Feet.of(6.7), Seconds.of(0.9))
      .put(Feet.of(10.4), Seconds.of(1.4))
      .put(Feet.of(15.2), Seconds.of(1.4))
      .put(Feet.of(20), Seconds.of(1.6));

  private final InterpolatingMeasureTreeMap<Distance, Angle> m_scoringHoodAngleMap = InterpolatingMeasureTreeMap
      .createDistanceToAngleMap()
      .put(Feet.of(4), Degrees.of(0))
      .put(Feet.of(5), Degrees.of(5 - 2))
      .put(Feet.of(6), Degrees.of(6.5 - 2))
      .put(Feet.of(7), Degrees.of(9 - 2))
      .put(Feet.of(8), Degrees.of(10 - 2))
      .put(Feet.of(9), Degrees.of(10 - 2))
      .put(Feet.of(10), Degrees.of(11 - 2))
      .put(Feet.of(11), Degrees.of(11 - 2))
      .put(Feet.of(12), Degrees.of(12 - 2))
      .put(Feet.of(13), Degrees.of(12 - 2))
      .put(Feet.of(14), Degrees.of(12 - 2))
      .put(Feet.of(15), Degrees.of(12 - 2))
      .put(Feet.of(16), Degrees.of(12 - 2));

  private final InterpolatingMeasureTreeMap<Distance, AngularVelocity> m_scoringFlywheelVelocityMap = InterpolatingMeasureTreeMap
      .createDistanceToAngularVelocityMap()
      .put(Feet.of(4), RPM.of(1450 + 50 + 25 + 50))
      .put(Feet.of(5), RPM.of(1450 + 50 + 25 + 50))
      .put(Feet.of(6), RPM.of(1500 + 50 + 25 + 50))
      .put(Feet.of(7), RPM.of(1600 + 50 + 25 + 50))
      .put(Feet.of(8), RPM.of(1600 + 50 + 25 + 50))
      .put(Feet.of(9), RPM.of(1700 + 50 + 25 + 50))
      .put(Feet.of(10), RPM.of(1700 + 50 + 25 + 50))
      .put(Feet.of(11), RPM.of(1800 + 50 + 50))
      .put(Feet.of(12), RPM.of(1850 + 25 + 50))
      .put(Feet.of(13), RPM.of(2000 + 25 + 50))
      .put(Feet.of(14), RPM.of(2050 + 25 + 50))
      .put(Feet.of(15), RPM.of(2100 + 25 + 50))
      .put(Feet.of(16), RPM.of(2200 + 25 + 50))
      .put(Feet.of(25), RPM.of(2600 + 25 + 50));

  public ShotCalculationUtil(Supplier<Transform3d> turretTransformSupplier,
      Supplier<Translation3d> targetTranslationSupplier,
      Supplier<Transform2d> robotTransformSupplier) {
    m_turretTransformSupplier = turretTransformSupplier;
    m_targetTranslationSupplier = targetTranslationSupplier;
    m_robotTransformSupplier = robotTransformSupplier;
  }

  public ShotParameters calculateShot() {
    var turretTransform = m_turretTransformSupplier.get();
    var targetTranslation = m_targetTranslationSupplier.get();
    var robotTransform = m_robotTransformSupplier.get();

    var turretAngle = getTurretAngle(turretTransform.getTranslation(), targetTranslation, robotTransform.getRotation());
    var hoodAngle = getHoodAngle(
        Meters.of(turretTransform.getTranslation().getDistance(targetTranslation)), m_scoringHoodAngleMap);
    var flywheelVelocity = getFlyWheelVelocity(
        Meters.of(turretTransform.getTranslation().getDistance(targetTranslation)), m_scoringFlywheelVelocityMap);

    return new ShotParameters(turretAngle, hoodAngle, flywheelVelocity);
  }

  private Optional<Translation2d> getVelocityCompensatedTarget(
      Translation2d turret, Translation2d target, ChassisVelocities turretSpeeds, Time ToF, Time oldToF,
      InterpolatingMeasureTreeMap<Distance, Time> tofMap, int iterations) {

    if (iterations > kMaximumIterations) {
      return Optional.empty();
    }

    // The ball inherits the turret's velocity at launch. To cancel this drift,
    // aim at a virtual target shifted opposite to the velocity vector.
    final Translation2d adjustedTarget = target.minus(
        new Translation2d(
            turretSpeeds.vx * ToF.in(Seconds),
            turretSpeeds.vy * ToF.in(Seconds)));

    if (ToF.isNear(oldToF, kTimeOfFlightTolerance)) {
      // Converged — return the adjusted target using the converged ToF
      return Optional.of(adjustedTarget);
    }

    // Refine ToF based on the distance to the adjusted target
    final Distance distanceToTarget = Meters.of(turret.getDistance(adjustedTarget));
    final Time newToF = getTimeOfFlight(distanceToTarget, tofMap);

    return getVelocityCompensatedTarget(turret, target, turretSpeeds, newToF, ToF, tofMap, iterations + 1);
  }

  private Time getTimeOfFlight(Distance distance, InterpolatingMeasureTreeMap<Distance, Time> map) {
    return map.get(distance);
  }

  private Angle getHoodAngle(Distance distance, InterpolatingMeasureTreeMap<Distance, Angle> map) {
    return map.get(distance);
  }

  private AngularVelocity getFlyWheelVelocity(Distance distance,
      InterpolatingMeasureTreeMap<Distance, AngularVelocity> map) {
    return map.get(distance);
  }

  private Angle getTurretAngle(Translation3d turret, Translation3d target, Rotation2d robotRotation) {
    return target.minus(turret).toTranslation2d()
        .getAngle().get()
        .minus(robotRotation)
        .getMeasure();
  }
}
