package frc.statebasedcontroller.sequence.fundamental.phase;

import java.util.List;

import frc.statebasedcontroller.sequence.fundamental.sequence.BaseSequence;
import frc.statebasedcontroller.subsystem.fundamental.state.ISubsystemState;

public interface ISequencePhase {

    static boolean requireSubsystems(BaseSequence<? extends ISequencePhase> sequence, List<ISubsystemState> states) {
        for (ISubsystemState state : states) {
            if (state.getSubsystem().isRequiredByAnother(sequence)) {
                return false;
            }
        }
        for (ISubsystemState state : states) {
            state.getSubsystem().require(sequence, state);
        }
        return true;
    }

    SequencePhase getPhase();
}