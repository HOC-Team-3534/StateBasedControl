package frc.statebasedcontroller.sequence.fundamental.sequence;

import frc.statebasedcontroller.sequence.fundamental.phase.ISequencePhase;
import frc.statebasedcontroller.sequence.fundamental.phase.SequencePhase;
import frc.statebasedcontroller.subsystem.fundamental.subsystem.BaseSubsystem;

/**
 * This base sequence class should be extended by all teleop sequences (auton sequences should extend {@link BaseAutonSequence} instead).
 * 
 * Sequences are a combination of {@link SequencePhase phases}. The phase can be switched using custom logic in {@link #process()}.
 * Sequences keep track of time for convenience.
 */
public abstract class BaseSequence<SeqP extends ISequencePhase> implements ISequence<SeqP> {

    final SeqP neutralPhase, startPhase;
    SeqP phase, nextPhase;
    long timeAtStartOfSequence, timeAtStartOfPhase;
    boolean phaseFirstRunThrough;

    /**
     * 
     * @param neutralPhase the phase, usually enum labeled "NETURAL", that has no required subsystems
     * @param startPhase the first phase to go to when the sequence begins running/processing
     */
    public BaseSequence(SeqP neutralPhase, SeqP startPhase) {
        this.neutralPhase = neutralPhase;
        this.startPhase = startPhase;
        setNextPhase(neutralPhase);
        setPhase(neutralPhase);
    }

    /**
     * Reset puts the phase in the neutral phase
     * 
     * @return successfully updated phase
     */
    public boolean reset() {
        setNextPhase(getNeutralPhase());
        return updatePhase();
    }

    /**
     * When the sequence is currently in its neutral phase,
     * call this method to begin the sequence at the start phase,
     * which will require the subsystems to go to the associated state
     * 
     * @param subsystems usually empty. subsystems that must abort their current sequences, using {@link BaseSubsystem#forceRelease() forceRelease}
     */
    public void start(BaseSubsystem... subsystems) {
        if (isNeutral()) {
            resetSequenceStartTime();
            for (BaseSubsystem subsystem : subsystems) {
                if (!subsystem.forceRelease()) {
                    return;
                }
            }
            setNextPhase(getStartPhase());
            setPhase(getStartPhase());
        }
    }

    /**
     * Must be called at the end of {@link #process()} in each sequence, otherwise the
     * phase will not change and the sequence will get stuck. If it is not placed at the
     * very end, the functionality will not work correctly for logic that requires the first run
     * through flag
     * 
     * @return phase sucessfully changed when the next phase was different
     */
    protected boolean updatePhase() {
        phaseFirstRunThrough = false;
        if (getPhase() != getNextPhase()) {
            return setPhase(nextPhase);
        }
        return false;
    }

    /**
     * This only returns the phase enum. For the phase information, call {@link ISequencePhase#getPhase()} on top of this getPhase() method to get the 
     * {@link SequencePhase} instance associated with the {@link ISequencePhase} enum.
     * 
     * @return the current phase enum. 
     */
    public SeqP getPhase() {
        return this.phase;
    }

    /**
     * Requires all the the subsubsystems associated with the phase and sets the phase if successful
     * 
     * @param phase the phase enum to switch the sequence to
     * @return successfully required subsystems and switch phase of sequence
     */
    boolean setPhase(SeqP phase) {
        if (phase.getPhase().requireSubsystems(this)) {
            this.phase = phase;
            resetPhaseStartTime();
            phaseFirstRunThrough = true;
            return true;
        }
        return false;
    }

    /**
     * 
     * @return the next phase of the sequence, which is used to direct the flow of phases within the sequence
     */
    SeqP getNextPhase() {
        return nextPhase;
    }    

    /**
     * 
     * @param phase the phase to switch the sequence to instead of the current phase to control flow of the sequence
     */
    protected void setNextPhase(SeqP phase) {
        nextPhase = phase;
    }

    /**
     * 
     * @return the phase of the sequence where no subsystems are required, set at initialization
     */
    SeqP getNeutralPhase() {
        return neutralPhase;
    }

    /**
     * 
     * @return the phase of the sequence to begin the flow of phases to control the states of the subsystems to control the robot, set at initialization
     */
    SeqP getStartPhase() {
        return startPhase;
    }

    /**
     * 
     * @return the time in milliseconds since the start of the sequence
     */
    public long getTimeSinceStartOfSequence() {
        return System.currentTimeMillis() - timeAtStartOfSequence;
    }

    /**
     * set the sequence start time to the current time milliseconds
     */
    void resetSequenceStartTime() {
        timeAtStartOfSequence = System.currentTimeMillis();
    }

    /**
     * 
     * @return the time in milliseconds since the start of the current phase of the sequence
     */
    public long getTimeSinceStartOfPhase() {
        return System.currentTimeMillis() - timeAtStartOfPhase;
    }

    /**
     * set the phase start time to the current time milliseconds
     */
    void resetPhaseStartTime() {
        timeAtStartOfPhase = System.currentTimeMillis();
    }

    /**
     * 
     * @return is it the first time the sequence has been ran since the phase has change to the current phase
     */
    public boolean getPhaseFirstRunThrough() {
        return phaseFirstRunThrough;
    }

    /**
     * 
     * @return is the sequence in its neutral phase
     */
    public boolean isNeutral() {
        return getPhase() == getNeutralPhase();
    }
}