package frc.statebasedcontroller.subsystem.fundamental.subsystem;

public interface ISubsystem {
    /**
     * called every loop of the code, typically covered by
     * {@link BaseSubsystem#process()} but can be customized for additional
     * functionality every loop of the subsystem
     */
    void process();

    /**
     * contains the calls to neutralize all components of the subsystem for the
     * robot to be safe
     */
    void neutral();

    /**
     * contains the calls to neturalize all components of the subsystem for the
     * robot to be safe from any unknown state. The subsystem may not be able to
     * stop at the moment in order to maintain the safety of people or its own
     * structural durability.
     * 
     * @return successfuly aborted the functionality of the robot pertaining to the
     *         subsystem.
     */
    boolean abort();
}