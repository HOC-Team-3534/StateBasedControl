package frc.robot.sequences.parent;


import frc.robot.subsystems.parent.BaseSubsystem;

public abstract class BaseSequence<SeqP extends ISequencePhase> implements ISequence<SeqP> {

    SeqP phase = null;
    SeqP nextPhase = null;
    SeqP neutralPhase = null;
    SeqP startPhase = null;

    long timeAtStartOfSequence = 0;
    long timeAtStartOfPhase = 0;

    boolean phaseFirstRunThrough = false;

    public BaseSequence(SeqP neutralPhase, SeqP startPhase) {
        setNeutralPhase(neutralPhase);
        setStartPhase(startPhase);
        setNextPhase(neutralPhase);
        setPhase(neutralPhase);
    }

    /**
     * must be called at the beginning of the child class start function
     */
    void init() {
        updateSequenceStartTime();
    }

    public boolean reset() {
        setNextPhase(getNeutralPhase());
        return updatePhase();
    }

    public void start() {
        if (isNeutral()) {
            init();
            setNextPhase(getStartPhase());
            setPhase(getStartPhase());
        }
    }

    public void start(BaseSubsystem... subsystems) {
        if (isNeutral()) {
            init();
            for (BaseSubsystem subsystem : subsystems) {
                if (!subsystem.forceRelease()) {
                    return;
                }
            }
            setNextPhase(getStartPhase());
            setPhase(getStartPhase());
        }
    }

    public boolean isNeutral() {
        return getPhase() == getNeutralPhase();
    }

    boolean setPhase(SeqP phase) {
        if (phase.getPhase().requireSubsystems(this)) {
            this.phase = phase;
            updateStateStartTime();
            phaseFirstRunThrough = true;
            return true;
        }
        return false;
    }

    public SeqP getPhase() {
        return this.phase;
    }

    SeqP getNextPhase() {
        return nextPhase;
    }

    protected void setNextPhase(SeqP state) {
        nextPhase = state;
    }

    public SeqP getNeutralPhase() {
        return neutralPhase;
    }

    void setNeutralPhase(SeqP state) {
        neutralPhase = state;
    }

    SeqP getStartPhase() {
        return startPhase;
    }

    void setStartPhase(SeqP state) {
        startPhase = state;
    }

    protected boolean updatePhase() {
        phaseFirstRunThrough = false;
        if (getPhase() != getNextPhase()) {
            return setPhase(nextPhase);
        }
        return false;
    }

    void updateSequenceStartTime() {
        timeAtStartOfSequence = System.currentTimeMillis();
    }

    public long getTimeSinceStartOfSequence() {
        return System.currentTimeMillis() - timeAtStartOfSequence;
    }

    void updateStateStartTime() {
        timeAtStartOfPhase = System.currentTimeMillis();
    }

    public long getTimeSinceStartOfPhase() {
        return System.currentTimeMillis() - timeAtStartOfPhase;
    }

    public boolean getPhaseFirstRunThrough() {
        return phaseFirstRunThrough;
    }

}