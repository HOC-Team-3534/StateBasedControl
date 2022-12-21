package frc.statebasedcontroller.subsystem.fundamental.subsystem;

import frc.statebasedcontroller.sequence.fundamental.phase.ISequencePhase;
import frc.statebasedcontroller.sequence.fundamental.sequence.BaseSequence;
import frc.statebasedcontroller.subsystem.fundamental.state.ISubsystemState;
import frc.statebasedcontroller.subsystem.fundamental.state.SubsystemState;
import frc.statebasedcontroller.subsystem.general.swervedrive.BaseDriveSubsystem;

/**
 * This base subsystem class should be extended by all subsystems of the robot or another 
 * base subsystem with additional functionality, such as {@link BaseDriveSubsystem} for a
 * swerve drive subsystem.
 * 
 * This class encapsulates safety to ensure multiple sequences do not control the components
 * of the same subsystem. Instead, only one sequence is allowed to control the subsystem at a
 * time using the system of requiring. Once a sequence has required a subsystem, it can require
 * the subystem to be in a particular state to produce some functionality described by that
 * states calls to components using the lamda expression in its {@link SubsystemState}.
 */
public abstract class BaseSubsystem<SsS extends ISubsystemState> implements ISubsystem {

    BaseSequence<? extends ISequencePhase> sequenceRequiring;
    boolean required, stateFirstRunThrough;

    final SsS neutralState;
    SsS currentSubsystemState;

    /**
     * create the instance of the subsystem and start off in the neutral state where nothing is moving on the robot.
     * 
     * @param neutralState the state, typically labeled NEUTRAL, where no calls are made to components unless to turn them off. 
     */
    public BaseSubsystem(SsS neutralState) {
        this.neutralState = neutralState;
        setCurrentSubsystemState(neutralState);
    }

    /**
     * called every loop of the robot code. checks if the subsystem is still required
     * followed by calling the lamda expression for the current state of the subsystem
     * which creates the functionality of the robot
     */
    public void process() {
        isStillRequired();
        getCurrentSubsystemState().getState().process();
        setStateFirstRunThrough(false);
    }

    /**
     * Checks if the subsystem is required by a different sequence. Used in the process of determining if the subsystem 
     * can be required to be in a state determined by the phase of a sequence
     * 
     * @param sequence the requiring sequence desiring to control the subsystem
     * @return is subsystem required by a different sequence
     */
    public boolean isRequiredByAnother(BaseSequence<? extends ISequencePhase> sequence) {
        if (sequenceRequiring == sequence) {
            return false;
        }
        return this.required;
    }

    /**
     * An attempt to require the subsystem by a sequence to set the state based on the the phase of that sequence
     * 
     * @param sequence  the requiring sequence desiring to control the subsystem
     * @param subsystemState the desired subsystem state according to the phase of the sequence
     * @return successfully required the subsystem
     */
    public boolean require(BaseSequence<? extends ISequencePhase> sequence, SsS subsystemState) {
        if (!isRequiredByAnother(sequence)) {
            required = true;
            setSequenceRequiring(sequence);
            setCurrentSubsystemState(subsystemState);
            return true;
        } else if (sequenceRequiring == sequence) {
            setCurrentSubsystemState(subsystemState);
            return true;
        } else {
            return false;
        }
    }

    /**
     * 
     * @return the requiring sequence is still running the phase that required the subsystem to be in the current state
     */
    boolean isStillRequired() {
        if (!required) {
            return false;
        } else if (!sequenceRequiring.getPhase().getPhase().getRequiredSubsystems().contains(this)) {
            release();
            return false;
        } else {
            return true;
        }
    }

    /**
     * cut ties with the sequence controlling the subsystem, and subsequentally neturalize the subsystem
     */
    void release() {
        required = false;
        sequenceRequiring = null;
        setCurrentSubsystemState(neutralState);
    }

    /**
     * abort the subsystem no matter the current requiring sequence and associated phase, typically used
     * during starting a sequence that overpowers another sequence (e.g. driving manually depending on logic)
     * 
     * @return successfully aborted subsystem and reset the requiring sequence, followed by releasing subsystem
     */
    public boolean forceRelease() {
        if (this.getSequenceRequiring() == null) {
            return true;
        }
        if (this.abort()) {
            if (this.getSequenceRequiring().reset()) {
                release();
                return true;
            }
        }
        return false;
    }

    /**
     * 
     * @return is it the first loop since the state has changed to the current state of the subsystem
     */
    public boolean getStateFirstRunThrough() {
        return this.stateFirstRunThrough;
    }

    /**
     * 
     * @param firstRun is it the first run of the subsystem since the state has been changed
     */
    void setStateFirstRunThrough(boolean firstRun){
        this.stateFirstRunThrough = firstRun;
    }

    /**
     * 
     * @return the sequence requiring the control of the subsystem
     */
    public BaseSequence<? extends ISequencePhase> getSequenceRequiring() {
        return sequenceRequiring;
    }

    /**
     * 
     * @param sequence the sequence that is requiring the control of the subsystem
     */
    void setSequenceRequiring(BaseSequence<? extends ISequencePhase> sequence) {
        this.sequenceRequiring = sequence;
    }

    /**
     * The current subsystem state is determine by the phase of the sequence requiring the subsystem.
     * 
     * @return the current subystem state
     */
    public SsS getCurrentSubsystemState() {
        return this.currentSubsystemState;
    }

    /**
     * 
     * @param state the subsystem state that the current phase of the sequence requiring the subsystem commands.
     */
    void setCurrentSubsystemState(SsS state) {
        setStateFirstRunThrough(true);
        this.currentSubsystemState = state;
    }
}
