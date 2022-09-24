package frc.statebasedcontroller.sequence.fundamental;

import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import frc.statebasedcontroller.subsystem.fundamental.BaseSubsystem;
import frc.statebasedcontroller.subsystem.fundamental.SubsystemRequirement;

public class SequencePhase {

    List<SubsystemRequirement> subsystemRequirements;
    Set<BaseSubsystem> requiredSubsystems;

    public SequencePhase(SubsystemRequirement... requirements) {
        subsystemRequirements = Arrays.asList(requirements);
        requiredSubsystems = subsystemRequirements.stream().map(requirement -> requirement.getSubsystem()).collect(Collectors.toSet());
    }

    public Set<BaseSubsystem> getRequiredSubsystems() {
        return requiredSubsystems;
    }

    public boolean requireSubsystems(BaseSequence<? extends ISequencePhase> sequence) {
        return ISequencePhase.requireSubsystems(sequence, subsystemRequirements);
    }

}