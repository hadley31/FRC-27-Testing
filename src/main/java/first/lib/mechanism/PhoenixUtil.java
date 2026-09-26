package first.lib.mechanism;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;

/** Retries a Phoenix 6 config-apply call until it reports success or the attempt budget runs out. */
public final class PhoenixUtil {
  private static final int MAX_ATTEMPTS = 5;

  private PhoenixUtil() {
  }

  public static void tryUntilOk(Supplier<StatusCode> configCall) {
    for (int i = 0; i < MAX_ATTEMPTS; i++) {
      if (configCall.get().isOK()) {
        return;
      }
    }
  }
}
