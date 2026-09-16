// Copyright (c) Choreo contributors

package choreo.auto;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.OpMode;

/** An OpMode that runs a routine selected from an AutoChooser. */
public class AutoChooserOpMode implements OpMode {
  private final AutoChooser autoChooser;
  private Command autonomousCommand;

  /**
   * Creates a new AutoChooserOpMode.
   *
   * @param autoChooser the AutoChooser to use for selecting the autonomous routine to run.
   */
  public AutoChooserOpMode(AutoChooser autoChooser) {
    this.autoChooser = autoChooser;
  }

  @Override
  public void start() {
    autonomousCommand = autoChooser.selectedCommand();

    if (autonomousCommand != null) {
      Scheduler.getDefault().schedule(autonomousCommand);
    }
  }

  @Override
  public void end() {
    if (autonomousCommand != null) {
      Scheduler.getDefault().cancel(autonomousCommand);
    }
  }
}
