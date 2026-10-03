package first.lib.mechanism;

import java.util.List;

import org.wpilib.command3.Mechanism;

public interface LoggedMultiComponentMechanism extends Mechanism {
  public List<? extends LoggedComponent<?, ?>> getComponents();

  public default void updateComponents() {
    getComponents().forEach(LoggedComponent::update);
  }
}
