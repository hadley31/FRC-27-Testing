package first.lib.mechanism;

import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;

/** A reusable bundle of kS/kV/kP/kI/kD gains published as tunables under a common name prefix. */
public class TunableGains {
  private final TunableDouble m_kS;
  private final TunableDouble m_kV;
  private final TunableDouble m_kP;
  private final TunableDouble m_kI;
  private final TunableDouble m_kD;

  public TunableGains(String namePrefix, double kS, double kV, double kP, double kI, double kD) {
    m_kS = Tunables.addDouble(namePrefix + " kS", kS);
    m_kV = Tunables.addDouble(namePrefix + " kV", kV);
    m_kP = Tunables.addDouble(namePrefix + " kP", kP);
    m_kI = Tunables.addDouble(namePrefix + " kI", kI);
    m_kD = Tunables.addDouble(namePrefix + " kD", kD);
  }

  public TunableGains(String namePrefix) {
    this(namePrefix, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  public double kS() {
    return m_kS.get();
  }

  public double kV() {
    return m_kV.get();
  }

  public double kP() {
    return m_kP.get();
  }

  public double kI() {
    return m_kI.get();
  }

  public double kD() {
    return m_kD.get();
  }

  public boolean hasChanged() {
    return m_kS.hasChanged() || m_kV.hasChanged() || m_kP.hasChanged() || m_kI.hasChanged() || m_kD.hasChanged();
  }
}
