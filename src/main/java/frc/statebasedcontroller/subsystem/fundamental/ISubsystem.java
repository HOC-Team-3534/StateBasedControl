package frc.statebasedcontroller.subsystem.fundamental;

public interface ISubsystem {

    void process();

    void neutral();

    boolean abort();

}