package frc.statebasedcontroller.subsystem.fundamental.state;

import frc.statebasedcontroller.subsystem.fundamental.subsystem.BaseSubsystem;

public interface ISubsystemState<BaseS extends BaseSubsystem> {

    SubsystemState<BaseS> getState();

    BaseS getSubsystem();
}
