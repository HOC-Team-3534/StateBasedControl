package frc.statebasedcontroller.subsystem.fundamental.state;

import java.util.function.Consumer;

import frc.statebasedcontroller.subsystem.fundamental.subsystem.BaseSubsystem;

/**
 * This subsystem state class encapsulates the call to the consumer function for
 * the given subsytem state The functionality of the physical robot is
 * controllable using the lambda expression given to the state to call methods
 * within the associated subsystem.
 */
public class SubsystemState<BaseS extends BaseSubsystem> {
    ISubsystemState<BaseS> state;
    Consumer<BaseS> processFunction;

    /**
     * @param state           the associated enum state
     * @param processFunction the lambda expression that takes the associated
     *                        subsystem as the argument and calls methods to control
     *                        the robot
     */
    public SubsystemState(ISubsystemState<BaseS> state, Consumer<BaseS> processFunction) {
        this.state = state;
        this.processFunction = processFunction;
    }

    /**
     * call the block of code defined in the lambda expression for the state, now
     * directly making calls to control the robot
     */
    public void process() {
        processFunction.accept(state.getSubsystem());
    }
}
