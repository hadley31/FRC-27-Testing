package first.robot.oi;

import org.wpilib.command3.Trigger;

public interface DriverControls {
  public double getDriveForward();

  public double getDriveLeft();

  public double getDriveRotate();

  Trigger setModeToScore();

  Trigger setModeToStow();
}
