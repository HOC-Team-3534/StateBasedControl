package frc.statebasedcontroller.sequence.fundamental;

import java.util.List;

import frc.statebasedcontroller.subsystem.fundamental.BaseSubsystem;
import frc.statebasedcontroller.subsystem.fundamental.ISubsystemState;
import frc.statebasedcontroller.subsystem.fundamental.SubsystemRequirement;

public interface ISequencePhase {

    static boolean requireSubsystems(BaseSequence<? extends ISequencePhase> sequence, List<SubsystemRequirement> requirements) {
        for (SubsystemRequirement subsystemRequirement : requirements) {
            if (subsystemRequirement.getSubsystem().isRequiredByAnother(sequence)) {
                return false;
            }
        }
        for (SubsystemRequirement subsystemRequirement : requirements) {
            subsystemRequirement.getSubsystem().require(sequence, getSubsystemState(subsystemRequirement.getSubsystem(), requirements));
        }
        return true;
    }

    static <BaseS extends BaseSubsystem, SsS extends ISubsystemState<BaseS>> SsS getSubsystemState(BaseS subsystem, List<SubsystemRequirement> requirements) {
        for (SubsystemRequirement requirement : requirements) {
            if (requirement.getSubsystem().equals(subsystem)) {
                return (SsS) requirement.getSubsystemState();
            }
        }
        return null;
    }

    SequencePhase getPhase();
}