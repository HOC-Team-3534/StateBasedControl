package frc.statebasedcontroller.subsystem.fundamental.state;

import frc.statebasedcontroller.subsystem.fundamental.subsystem.BaseSubsystem;

public interface ISubsystemState<BaseS extends BaseSubsystem> {

    /**
     * 
     * @return the instance of {@link BaseSubsystem} associated with the enum of the state for the subystem
     */
    BaseSubsystem getState();

    /**
     * 
     * @return associated subsystem to the states
     */
    BaseS getSubsystem();
}
