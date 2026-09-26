// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command3.Scheduler;
import org.wpilib.epilogue.Logged;

import com.ctre.phoenix6.CANBus;

import first.lib.LoggedOpModeRobot;
import first.lib.mechanism.angle.TalonFXAngleMechanismIO;
import first.lib.mechanism.angularvelocity.TalonFXAngularVelocityMechanismIO;
import first.robot.mechanism.drive.Drive;
import first.robot.mechanism.feeder.Feeder;
import first.robot.mechanism.flywheel.Flywheel;
import first.robot.mechanism.hood.Hood;
import first.robot.mechanism.turret.Turret;
import first.robot.util.Constants;

@Logged
public class Robot extends LoggedOpModeRobot {
  private static final CANBus CAN_BUS = new CANBus("CANivore");

  public final Drive drive;
  public final Flywheel flywheel;
  public final Turret turret;
  public final Hood hood;
  public final Feeder feeder;

  public Robot() {
    // AdvantageKit must be configured and started before anything else is constructed, so that IO
    // implementations can log or replay their first set of inputs on the very first cycle.
    configureLogging();

    drive = new Drive();
    flywheel = new Flywheel(new TalonFXAngularVelocityMechanismIO(0, CAN_BUS));
    turret = new Turret(new TalonFXAngleMechanismIO(1, CAN_BUS));
    hood = new Hood(new TalonFXAngleMechanismIO(2, CAN_BUS));
    feeder = new Feeder(new TalonFXAngularVelocityMechanismIO(3, CAN_BUS));
  }

  private void configureLogging() {
    Logger.recordMetadata("ProjectName", "FRC-Testing-2027");
    Logger.recordMetadata("RuntimeType", getRuntimeType().toString());

    switch (Constants.kCurrentMode) {
      case REAL -> {
        // Log to a USB stick ("/U/logs") and publish live data to NetworkTables.
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
      }
      case SIM -> Logger.addDataReceiver(new NT4Publisher());
      case REPLAY -> {
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
      }
    }

    Logger.start();
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getDefault().run();
  }
}
