package frc;

public interface ISubsystemState<BaseS extends BaseSubsystem> {

    SubsystemState<BaseS> getState();
}
