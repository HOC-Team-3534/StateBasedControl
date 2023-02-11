package frc.pathplanner;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.pathplanner.config.PathPlannerConfig;

/**
 * The core of being able to follow a {@link PathPlannerTrajectory} from a .path
 * file from PathPlanner. The path is generated upon initialization using the
 * {@link #loadPath(String, double, double) loadPath(String pathName, double
 * maxSpeed, double maxAccel)} method. Use the {@link #getInitialState()} in
 * order to see the initial {@link PathPlannerState} Make sure to
 * {@link #resetStart()} when ready to follow the path, then
 * {@link #getCurrentState()}
 */
public class PathPlannerFollower {
    private PathPlannerTrajectory path;
    private long START_TIME;

    /**
     * @param pathName      the literal name of the path file, without the
     *                      extension
     * @param autonMaxSpeed the max speed of the robot while following the path
     *                      during autonomous
     * @param autonMaxAccel the max acceleration of the robot while following
     *                      the path during autonomous
     */
    public PathPlannerFollower(String pathName, double autonMaxSpeed,
                               double autonMaxAccel) {
        long load_start = System.currentTimeMillis();
        loadPath(pathName, autonMaxSpeed, autonMaxAccel);
        System.out.println(String.format("Path: [%s] took %d milliseconds.", pathName, System.currentTimeMillis() - load_start));
    }

    public PathPlannerFollower(Pose2d currPose, ChassisSpeeds currSpeeds,
                               Pose2d endPose, Rotation2d endHeading,
                               double endVelocity, double autonMaxSpeed,
                               double autonMaxAccel) {
        PathPoint startPoint;
        if (new Translation2d(currSpeeds.vxMetersPerSecond,
                              currSpeeds.vyMetersPerSecond).getNorm() + currSpeeds.omegaRadiansPerSecond <= 0.05) {
            startPoint = new PathPoint(currPose.getTranslation(),
                                       endPose.getTranslation().minus(currPose.getTranslation()).getAngle(),
                                       currPose.getRotation());
        } else {
            startPoint = PathPoint.fromCurrentHolonomicState(currPose, currSpeeds);
        }
        this.path = PathPlanner.generatePath(new PathConstraints(autonMaxSpeed,
                                                                 autonMaxAccel), startPoint, new PathPoint(endPose.getTranslation(),
                                                                                                           endHeading,
                                                                                                           endPose.getRotation(),
                                                                                                           endVelocity));
    }

    /**
     * Load the path for the path follower from the file
     * 
     * @param pathName the literal name of the path file, without the extension
     * @param maxSpeed the max speed of the robot while following the path
     *                 during autonomous
     * @param mxAccel  the max acceleration of the robot while following the
     *                 path during autonomous
     */
    private void loadPath(String pathName, double maxSpeed, double maxAccel) {
        this.path = PathPlanner.loadPath(pathName, maxSpeed, maxAccel);
    }

    /**
     * Get the state the robot will at, at a particular time
     * 
     * @param seconds the time at which to determine the state of the robot
     *                given the path
     * 
     * @return the state the robot will be at, at the particular time
     */
    public PathPlannerTrajectory.PathPlannerState getState(double seconds) {
        return (PathPlannerTrajectory.PathPlannerState) path.sample(seconds);
    }

    /**
     * Given the time since the start of following the path, the state the robot
     * should currently be at
     * 
     * @return the current state the robot should be at
     */
    public PathPlannerTrajectory.PathPlannerState getCurrentState() {
        double timeSinceStart
                        = (double) (System.currentTimeMillis() - START_TIME) / 1000.0;
        return (PathPlannerTrajectory.PathPlannerState) path.sample(timeSinceStart + PathPlannerConfig.LOOK_AHEAD_TIME);
    }

    /**
     * Shift the time being used to sample the trajectory to the closest point
     * on the path within a window of time, set in {@link PathPlannerConfig}
     * 
     * @param robotPose the current pose of the robot
     */
    public void shiftTimeToClosestPoint(Pose2d robotPose) {
        double currTime = getClosestStateTime(robotPose);
        START_TIME += (int) (currTime * 1000) - getTimeSinceStart();
    }

    int getIndexForTime(PathPlannerTrajectory traj, double time) {
        int low = 0;
        int high = traj.getStates().size() - 1;
        while (low != high) {
            int mid = (low + high) / 2;
            if (traj.getState(mid).timeSeconds < time) {
                low = mid + 1;
            } else {
                high = mid;
            }
        }
        return low;
    }

    protected double getClosestStateTime(Pose2d robotPose) {
        int index = 0;
        int totalSize = this.path.getStates().size();
        double lowestDist
                        = this.path.getState(0).poseMeters.getTranslation().getDistance(robotPose.getTranslation());
        for (int i = 1; i < totalSize; i += 100) {
            double dist = this.path.getState(i).poseMeters.getTranslation().getDistance(robotPose.getTranslation());
            if (dist < lowestDist) {
                index = i;
                lowestDist = dist;
            }
        }
        int filteredStart = (index - 100 < 0) ? 0 : index - 100;
        int filteredEnd = (index + 100 > totalSize) ? totalSize : index + 100;
        for (int i = filteredStart; i < filteredEnd; i += 2) {
            double dist = this.path.getState(i).poseMeters.getTranslation().getDistance(robotPose.getTranslation());
            if (dist < lowestDist) {
                index = i;
                lowestDist = dist;
            }
        }
        return this.path.getState(index).timeSeconds;
    }

    /**
     * @return the initial state of the trajectory
     */
    public PathPlannerState getInitialState() {
        return path.getInitialState();
    }

    /**
     * @return the time since the start of following the path in seconds
     */
    public double getTimeSinceStart() {
        return (double) (System.currentTimeMillis() - START_TIME) / 1000.0;
    }

    /**
     * @return the remaining time to completely follow the path
     */
    public double getRemainingTimeSeconds() {
        return path.getTotalTimeSeconds() - getTimeSinceStart();
    }

    /**
     * reset the start time of the path to the current time. needed in order to
     * create path then follow it later, so the time must be rest.
     */
    public void resetStart() {
        START_TIME = System.currentTimeMillis();
    }

    /**
     * @return has the time since the start met or exceed the duration of time
     *         of the path
     */
    public boolean isFinished() {
        double timeSinceStart
                        = (double) (System.currentTimeMillis() - START_TIME) / 1000.0;
        return timeSinceStart >= path.getTotalTimeSeconds();
    }
}
