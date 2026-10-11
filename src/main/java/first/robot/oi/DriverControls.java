package first.robot.oi;

import org.wpilib.command3.Trigger;

public interface DriverControls {
  public double getDriveForward();

  public double getDriveLeft();

  public double getDriveRotate();

  Trigger setModeToStow();

  Trigger setModeToIntake();

  Trigger setModeToScore();

  /**
   * Hands the aiming between the turret and the chassis. See {@code Tuning#kFixedTurretMode}.
   *
   * <p>A driver control as well as a dashboard toggle, because the reason to switch is usually
   * something only the driver can see -- a turret that has stopped responding, or a shot that keeps
   * missing to one side.
   */
  Trigger toggleFixedTurretMode();
}
