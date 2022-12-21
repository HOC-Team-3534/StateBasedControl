package frc.statebasedcontroller.sequence.fundamental.phase;

import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import frc.pathplanner.PathPlannerFollower;
import frc.statebasedcontroller.sequence.fundamental.sequence.BaseSequence;
import frc.statebasedcontroller.subsystem.fundamental.state.ISubsystemState;
import frc.statebasedcontroller.subsystem.fundamental.state.SubsystemState;
import frc.statebasedcontroller.subsystem.fundamental.subsystem.BaseSubsystem;

/**
 * This sequence phase class encapsulates the required {@link BaseSubsystem subsystems},
 * the associated {@link SubsystemState states}, and the {@link PathPlannerFollower path}.
 * 
 * The required subsystems are here, so {@link #requireSubsystems(BaseSequence) requireSubsystems}
 * is in this class.
 */
public class SequencePhase {

    final Supplier<PathPlannerFollower> pathGetter;
    List<ISubsystemState> subsystemStates;
    Set<BaseSubsystem> requiredSubsystems;

    /**
     * 
     * @param states all the states for each subsystem that they should be in during this phase of the sequence
     */
    public SequencePhase(ISubsystemState... states) {
        subsystemStates = Arrays.asList(states);
        pathGetter = () -> null;
    }

    /**
     * 
     * @param pathGetter the supplier of the autonomous path to follow during this phase of the sequence
     * @param states all the states for each subsystem that they should be in during this phase of the sequence
     */
    public SequencePhase(Supplier<PathPlannerFollower> pathGetter, ISubsystemState... states) {
        this.pathGetter = pathGetter;
        subsystemStates = Arrays.asList(states);
    }

    /**
     * 
     * @return the set of subsystems required by the phase
     */
    public Set<BaseSubsystem> getRequiredSubsystems() {
        return requiredSubsystems = subsystemStates.stream().map(state -> state.getSubsystem()).collect(Collectors.toSet());
    }

    /**
     * Requires all of the subsystems by the associated sequence for this phase
     * 
     * @param sequence the associated sequence of the phase
     * @return true if require is successful
     */
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

    /**
     * 
     * @return autonomous path followed during the phase
     */
    public PathPlannerFollower getPath() {
        return pathGetter.get();
    }
}
