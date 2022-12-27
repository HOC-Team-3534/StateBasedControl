package frc.statebasedcontroller.sequence.fundamental.phase;

/**
 * This sequence phase interface must be implemented by each enum of sequence
 * phases. These enums define the phases (aka steps) of the sequence. Each enum
 * has a {@link SequencePhase} that can be accessed using {@link #getPhase()}
 */
public interface ISequencePhase {
    /**
     * @return phase instance associated with the {@link ISequencePhase} enum
     */
    SequencePhase getPhase();
}