package frc.robot.subsystems.parent;

public interface ISubsystemState<BaseS extends BaseSubsystem> {

    SubsystemState<BaseS> getState();
}
