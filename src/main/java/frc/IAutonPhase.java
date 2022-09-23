package frc.robot.autons.parent;

import frc.robot.autons.pathplannerfollower.PathPlannerFollower;
import frc.robot.sequences.parent.ISequencePhase;

public interface IAutonPhase extends ISequencePhase {

    static PathPlannerFollower getPath(BaseAutonSequence<? extends IAutonPhase> sequence, int pathIndex) {
        if (pathIndex >= 0 && pathIndex < sequence.getPaths().size()) {
            return sequence.getPaths().get(pathIndex);
        }
        System.out.println("ERROR: Tried to get path for state that doesn't have a valid path");
        return null;
    }

    PathPlannerFollower getPath(BaseAutonSequence<? extends IAutonPhase> sequence);

}
