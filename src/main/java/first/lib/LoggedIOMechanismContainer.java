package first.lib;

import java.util.List;

import first.lib.mechanism.LoggedInputContainer;

public interface LoggedIOMechanismContainer {
  public List<? extends LoggedInputContainer<?, ?>> getIOContainers();

  public default void updateIOs() {
    getIOContainers().forEach(LoggedInputContainer::update);
  }
}
