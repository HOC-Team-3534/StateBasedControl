package frc;

public interface ISequence<SeqS extends ISequencePhase> {

    void process();

    SeqS getPhase();

    /**
     * @return when true, it is safe to control the mechanisms previously being controlled by the function
     */
    boolean abort();

}