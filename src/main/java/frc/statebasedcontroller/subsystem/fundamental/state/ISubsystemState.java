package frc.statebasedcontroller.subsystem.fundamental.state;

import frc.statebasedcontroller.subsystem.fundamental.subsystem.BaseSubsystem;

public interface ISubsystemState<BaseS extends BaseSubsystem> {
    /**
     * @return the instance of the associated subsystem state instance
     */
    SubsystemState<BaseS> getState();

    /**
     * @return associated subsystem to the states
     */
    BaseS getSubsystem();
}
