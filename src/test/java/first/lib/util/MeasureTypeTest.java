package first.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Rotations;

import org.junit.jupiter.api.Test;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Voltage;

/**
 * The point of pairing a unit with both of its conversions is that a value cannot change meaning on a
 * round trip, so that is what these check.
 */
class MeasureTypeTest {
  @Test
  void aMeasureInAnotherUnitIsConvertedRatherThanStrippedOfItsUnit() {
    // The failure this prevents: reading a magnitude straight off the measure, which would make one
    // rotation the number 1 and then read it back as a single degree.
    assertEquals(360.0, UnitDoubleConverter.DEGREES.toDouble(Rotations.of(1)), 1e-9);
  }

  @Test
  void aDoubleRoundTripsThroughTheUnitItWasReadIn() {
    assertEquals(13.5, UnitDoubleConverter.FEET.toDouble(UnitDoubleConverter.FEET.of(13.5)), 1e-9);
  }

  @Test
  void theMeasureCarriesTheUnitItWasBuiltWith() {
    assertEquals(36.0, UnitDoubleConverter.FEET.of(3.0).in(Inches), 1e-9);
  }

  @Test
  void aUnitTheCatalogDoesNotNameWorksTheSameWay() {
    // of() is generic over every WPILib unit, which is what keeps the catalog short rather than
    // exhaustive -- and what the unchecked cast inside it is there for.
    UnitDoubleConverter<Voltage> volts = UnitDoubleConverter.of(Units.Volts);

    assertEquals(12.0, volts.toDouble(volts.of(12.0)), 1e-9);
    assertEquals(Units.Volts, volts.unit());
  }
}
