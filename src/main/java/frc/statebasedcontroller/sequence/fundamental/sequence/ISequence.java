package frc.statebasedcontroller.sequence.fundamental.sequence;

import frc.statebasedcontroller.sequence.fundamental.phase.ISequencePhase;

public interface ISequence<SeqS extends ISequencePhase> {
    /**
     * place a switch on the enum for the {@link ISequencePhase phases} in this
     * method to switch the phase of the sequence, followed by a call to
     * {@link BaseSequence#updatePhase()} to make sure the phase changes
     */
    void process();

    /**
     * @return when true, it is safe to control the mechanisms previously being
     *         controlled by the function
     */
    boolean abort();
}