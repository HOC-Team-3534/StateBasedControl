package frc.statebasedcontroller.subsystem.fundamental;

public abstract class SubsystemRequirement<BaseS extends BaseSubsystem, SsS extends ISubsystemState<BaseS>> {

    BaseS subsystem;
    SsS subsystemState;

    public SubsystemRequirement(BaseS subsystem, SsS subsystemState) {
        this.subsystem = subsystem;
        this.subsystemState = subsystemState;
    }

    public BaseS getSubsystem() {
        return subsystem;
    }

    public SsS getSubsystemState() {
        return subsystemState;
    }
}
