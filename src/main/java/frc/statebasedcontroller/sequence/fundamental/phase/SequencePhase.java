package frc.statebasedcontroller.sequence.fundamental.phase;

import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import frc.pathplanner.PathPlannerFollower;
import frc.statebasedcontroller.sequence.fundamental.sequence.BaseAutonSequence;
import frc.statebasedcontroller.sequence.fundamental.sequence.BaseSequence;
import frc.statebasedcontroller.subsystem.fundamental.state.ISubsystemState;
import frc.statebasedcontroller.subsystem.fundamental.subsystem.BaseSubsystem;

public class SequencePhase {

    int pathIndex = -999;
    List<ISubsystemState> subsystemStates;
    Set<BaseSubsystem> requiredSubsystems;

    public SequencePhase(ISubsystemState... states) {
        subsystemStates = Arrays.asList(states);
    }

    public SequencePhase(int pathIndex, ISubsystemState... states) {
        this.pathIndex = pathIndex;
        subsystemStates = Arrays.asList(states);
    }

    public Set<BaseSubsystem> getRequiredSubsystems() {
        return requiredSubsystems = subsystemStates.stream().map(state -> state.getSubsystem()).collect(Collectors.toSet());
    }

    public boolean requireSubsystems(BaseSequence<? extends ISequencePhase> sequence) {
        for (ISubsystemState state : subsystemStates) {
            if (state.getSubsystem().isRequiredByAnother(sequence)) {
                return false;
            }
        }
        for (ISubsystemState state : subsystemStates) {
            state.getSubsystem().require(sequence, state);
        }
        return true;
    }

    public PathPlannerFollower getPath(BaseAutonSequence<? extends ISequencePhase> sequence) {
        if (pathIndex >= 0 && pathIndex < sequence.getPaths().size()) {
            return sequence.getPaths().get(pathIndex);
        }
        System.out.println("ERROR: Tried to get path for state that doesn't have a valid path");
        return null;
    }

}
