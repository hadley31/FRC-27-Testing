package first.lib.mechanism;

import java.util.List;

public interface LoggedMultiComponentMechanism {
  public List<? extends LoggedComponent<?, ?>> getComponents();

  public default void updateComponents() {
    getComponents().forEach(LoggedComponent::update);
  }
}
