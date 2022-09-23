package frc.robot.subsystems.parent;

import java.util.function.Consumer;

public class SubsystemState<BaseS extends BaseSubsystem> {

    String stateName;
    Consumer<BaseS> processFunction;
    BaseS subsystem;

    public SubsystemState(String stateName, Consumer<BaseS> processFunction, BaseS subsystem) {
        this.stateName = stateName;
        this.processFunction = processFunction;
        this.subsystem = subsystem;
    }

    public String getName() {
        return stateName;
    }

    public void process(BaseS subsystem) {
        this.subsystem = subsystem;
        process();
    }

    public void process() {
        processFunction.accept(subsystem);
    }
}
