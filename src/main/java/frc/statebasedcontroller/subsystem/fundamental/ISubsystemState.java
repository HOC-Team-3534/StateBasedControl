package frc.statebasedcontroller.subsystem.fundamental;

public interface ISubsystemState<BaseS extends BaseSubsystem> {

    SubsystemState<BaseS> getState();
}
