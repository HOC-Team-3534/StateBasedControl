package frc.statebasedcontroller.subsystem.fundamental.subsystem;

public interface ISubsystem {

    void process();

    void neutral();

    boolean abort();

}