// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.command3.Scheduler;
import org.wpilib.epilogue.Epilogue;
import org.wpilib.epilogue.Logged;
import org.wpilib.framework.OpModeRobot;

import com.ctre.phoenix6.CANBus;

import first.lib.mechanism.angle.TalonFXAngleMechanismIO;
import first.lib.mechanism.angularvelocity.TalonFXAngularVelocityMechanismIO;
import first.robot.mechanism.drive.Drive;
import first.robot.mechanism.feeder.Feeder;
import first.robot.mechanism.flywheel.Flywheel;
import first.robot.mechanism.hood.Hood;
import first.robot.mechanism.turret.Turret;
import first.robot.util.SchedulerLogger;

@Logged
public class Robot extends OpModeRobot {
  private static final CANBus CAN_BUS = new CANBus("CANivore");

  public final Drive drive;
  public final Flywheel flywheel;
  public final Turret turret;
  public final Hood hood;
  public final Feeder feeder;

  public Robot() {
    drive = new Drive();
    flywheel = new Flywheel(new TalonFXAngularVelocityMechanismIO(0, CAN_BUS));
    turret = new Turret(new TalonFXAngleMechanismIO(1, CAN_BUS));
    hood = new Hood(new TalonFXAngleMechanismIO(2, CAN_BUS));
    feeder = new Feeder(new TalonFXAngularVelocityMechanismIO(3, CAN_BUS));
  }

  @Override
  public void robotPeriodic() {
    Scheduler.getDefault().run();
    SchedulerLogger.refresh(Scheduler.getDefault());
    Epilogue.update(this);
  }
}
