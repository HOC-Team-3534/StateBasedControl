package frc.statebasedcontroller.subsystem.fundamental.state;

import java.util.function.Consumer;

import frc.statebasedcontroller.subsystem.fundamental.subsystem.BaseSubsystem;

public class SubsystemState<BaseS extends BaseSubsystem> {

    ISubsystemState<BaseS> state;
    Consumer<BaseS> processFunction;

    public SubsystemState(ISubsystemState<BaseS> state, Consumer<BaseS> processFunction) {
        this.state = state;
        this.processFunction = processFunction;
    }

    public void process() {
        processFunction.accept(state.getSubsystem());
    }
}
