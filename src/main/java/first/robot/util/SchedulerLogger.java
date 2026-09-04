package first.robot.util;

import org.wpilib.command3.Scheduler;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.ProtobufPublisher;

public class SchedulerLogger {
  private static final ProtobufPublisher<Scheduler> m_schedulerPublisher = NetworkTableInstance.getDefault()
      .getProtobufTopic("/Telemetry/Scheduler", Scheduler.proto)
      .publish();

  public static void refresh(Scheduler scheduler) {
    m_schedulerPublisher.set(scheduler);
  }
}
