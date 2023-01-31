package frc.statebasedcontroller.subsystem.general.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.pathplanner.PathPlannerFollower;
import frc.pathplanner.config.PathPlannerConfig;
import frc.statebasedcontroller.subsystem.fundamental.state.ISubsystemState;
import frc.statebasedcontroller.subsystem.fundamental.subsystem.BaseSubsystem;
import frc.statebasedcontroller.subsystem.general.swervedrive.swervelib.SwerveDrivetrainModel;
import frc.statebasedcontroller.subsystem.general.swervedrive.swervelib.SwerveInput;
import frc.statebasedcontroller.subsystem.general.swervedrive.swervelib.SwerveModule;

import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.Callable;

import org.ejml.simple.SimpleMatrix;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

/**
 * An extension of {@link BaseSubsystem}, the base drive subsystem is a
 * subsystem for swerve drive robots. It accomodates for the additional
 * functionality required by the control system of the swerve drive modules as a
 * whole. It can track the pose of the robot, follow a path autonomously, and
 * control the robot manually.
 */
public abstract class BaseDriveSubsystem<SsS extends ISubsystemState> extends BaseSubsystem<SsS> {
    static Rotation2d targetShootRotationAngle = new Rotation2d();
    final SwerveDrivetrainModel dt;
    final SwerveDriveKinematics kinematics;
    PathPlannerFollower pathPlannerFollower;
    FeedForwardCharacterizationData characterizationData
                    = new FeedForwardCharacterizationData();

    /**
     * @param dt           the drivetrain model that encompasses the modules and
     *                     allows for tracking and control
     * @param kinematics   the physical locations of the swerve modules relative
     *                     to the center of the robot
     * @param neutralState the state, typically labeled NEUTRAL, where no calls
     *                     are made to components unless to turn them off.
     */
    public BaseDriveSubsystem(SwerveDrivetrainModel dt,
                              SwerveDriveKinematics kinematics,
                              SsS neutralState) {
        super(neutralState);
        this.dt = dt;
        this.kinematics = kinematics;
        Timer.delay(1.0);
        for (SwerveModule mod : dt.getSwerveModules()) {
            mod.resetToAbsolute();
        }
    }

    /**
     * called every loop of the robot code. checks if the subsystem is still
     * required followed by calling the lamda expression for the current state
     * of the subsystem which creates the functionality of the robot
     * additionally, the drive subsystem keeps track of telemetry, send the
     * desired states of the modules to the modules and updates the simulation
     * if in simulation mode
     */
    public void process() {
        dt.updateOdometry();
        super.process();
    }

    /**
     * @return the path currently being followed by the drive subsystem during
     *         autonomous
     */
    public PathPlannerFollower getPathPlannerFollower() {
        return pathPlannerFollower;
    }

    /**
     * Set the desired {@link PathPlannerFollower} to follow during autonomous
     * 
     * @param ppf                the desired path follower to follow during
     *                           phase of autonomous sequence
     * @param setInitialPosition should the pose of the robot be reset to the
     *                           initial pose of the path
     */
    public void setPathPlannerFollower(PathPlannerFollower ppf,
                                       boolean setInitialPosition) {
        this.pathPlannerFollower = ppf;
        if (setInitialPosition) {
            dt.setKnownState(ppf.getInitialState());
        }
    }

    /**
     * @return get velocity vector of the robot with respect to the robot in
     *         meters per second (m/s)
     */
    public Translation2d getRobotCentricVelocity() {
        ChassisSpeeds chassisSpeeds
                        = kinematics.toChassisSpeeds(dt.getSwerveModuleStates());
        return new Translation2d(chassisSpeeds.vxMetersPerSecond,
                                 chassisSpeeds.vyMetersPerSecond);
    }

    /**
     * @return get velocity vecotr of the robot with respect to the target in
     *         meters per second (m/s)
     */
    public Translation2d getTargetOrientedVelocity() {
        Translation2d robotCentricVelocity = getRobotCentricVelocity();
        robotCentricVelocity.rotateBy(Rotation2d.fromDegrees(180.0));
        return robotCentricVelocity;
    }

    /**
     * @return the rotational error to aim the robot at the target
     */
    public Rotation2d getTargetShootRotationAngleError() {
        return targetShootRotationAngle.minus(dt.getPose().getRotation());
    }

    /**
     * Set the rotation the robot should turn to in order to aim the front at
     * the target
     * 
     * @param angleErrorFunction typically using vision, the function that
     *                           returns the offset to aim at target
     */
    public void setTargetShootRotationAngle(Callable<Rotation2d> angleErrorFunction) {
        try {
            targetShootRotationAngle
                            = dt.getPose().getRotation().plus(angleErrorFunction.call());
        } catch (Exception e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    /**
     * @return the current pose of the robot
     */
    public Pose2d getPose() {
        return dt.getPose();
    }

    /**
     * @return the current gyroscope rotation
     */
    public Rotation2d getGyroRotation() {
        return dt.getGyroHeading();
    }

    /**
     * reset the gyroscope rotation to zero to indicate the front is pointing
     * straight ahead
     */
    public void resetGyro() {
        dt.zeroGyroscope(new Rotation2d());
    }

    /**
     * @return the current states of the swerve modules from the drivetrain
     *         model
     */
    public SwerveModuleState[] getSwerveModuleStates() {
        return dt.getSwerveModuleStates();
    }

    /**
     * Set the desired module states based on desired chassis speeds
     * 
     * @param chassisSpeeds the speed in each resprect that the robot that is
     *                      currently desired
     */
    protected void setModuleStates(ChassisSpeeds chassisSpeeds,
                                   boolean isOpenLoop) {
        dt.setModuleStates(chassisSpeeds, isOpenLoop);
    }

    /**
     * Set the desired module states based on proportional inputs, typically
     * from a controller or similar device
     * 
     * @param x   the proportion (-1 to 1) in the backward and forward direction
     *            to drive the robot
     * @param y   the proportion (-1 to 1) in the left and right direction to
     *            drive the robot
     * @param rot the proportion (-1 to 1) in the counterclockwise and clockwise
     *            direction to drive the robot
     */
    protected void setModuleStates(double x, double y, double rot,
                                   boolean isOpenLoop) {
        dt.setModuleStates(new SwerveInput(x, y, rot), false, isOpenLoop);
    }

    /**
     * Set the desired module states based on proportional inputs, typically
     * from a controller or similar device, but with creep proportions
     * 
     * @param x   the proportion (-1 to 1) in the backward and forward direction
     *            to drive the robot cut to creep proportion
     * @param y   the proportion (-1 to 1) in the left and right direction to
     *            drive the robot cut to creep proportion
     * @param rot the proportion (-1 to 1) in the counterclockwise and clockwise
     *            direction to drive the robot cut to creep proportion
     */
    protected void setModuleStatesCreep(double x, double y, double rot,
                                        boolean isOpenLoop) {
        dt.setModuleStates(new SwerveInput(x, y, rot), true, isOpenLoop);
    }

    /**
     * Based on the current state of the path follower and the current pose, the
     * holonomic controller of the drivetrain model will calculate the desired
     * module states
     */
    public void setModuleStatesAutonomous() {
        PathPlannerState currState
                        = this.getPathPlannerFollower().getCurrentState();
        if (currState.poseMeters.getTranslation().getDistance(getPose().getTranslation()) > 0.150
            && PathPlannerConfig.USE_DIST_CORRECTION_MODE) {
            this.getPathPlannerFollower().shiftTimeToClosestPoint(getPose());
            currState = this.getPathPlannerFollower().getCurrentState();
        }
        dt.goToPose(currState);
    }

    protected void characterizeSteerInit() {
        dt.setVoltageToZero();
        characterizationData = new FeedForwardCharacterizationData();
    }

    protected void characterizeSteer(double voltage) {
        var fl = dt.getSwerveModules()[0];
        fl.setSteerVoltageForCharacterization(voltage);
        var data = fl.getSteerVoltageAndRate();
        characterizationData.add(data.getFirst(), data.getSecond());
    }

    protected void characterizeDriveInit() {
        dt.setVoltageToZero();
        characterizationData = new FeedForwardCharacterizationData();
    }

    protected void characterizeDrive(double voltage) {
        for (int i = 0; i < SwerveDrivetrainModel.NUM_MODULES; i++) {
            dt.getSwerveModules()[i].setDriveVoltageForCharacterization(voltage);
        }
        var fl = dt.getSwerveModules()[0];
        var data = fl.getDriveVoltageAndRate();
        characterizationData.add(data.getFirst(), data.getSecond());
    }

    public void printData() {
        characterizationData.print();
    }
}

class FeedForwardCharacterizationData {
    private final List<Double> velocityData = new LinkedList<>();
    private final List<Double> voltageData = new LinkedList<>();

    public void add(double velocity, double voltage) {
        if (Math.abs(velocity) > 1E-4) {
            velocityData.add(Math.abs(velocity));
            voltageData.add(Math.abs(voltage));
        }
    }

    public void print() {
        double velocityDataArray[]
                        = velocityData.stream().mapToDouble(Double::doubleValue).toArray();
        double voltageDataArray[]
                        = voltageData.stream().mapToDouble(Double::doubleValue).toArray();
        double accelerationDataArray[] = new double[velocityDataArray.length];
        for (int i = 0; i < velocityDataArray.length - 1; i++) {
            accelerationDataArray[i]
                            = (velocityDataArray[i + 1] - velocityDataArray[i]) / 0.020;
        }
        accelerationDataArray[accelerationDataArray.length - 1]
                        = accelerationDataArray[accelerationDataArray.length - 2];
        PolynomialRegression regression
                        = new PolynomialRegression(velocityDataArray,
                                                   voltageDataArray, 2);
        double residualsVoltageVelocityWise[]
                        = new double[velocityDataArray.length];
        for (int i = 0; i < velocityDataArray.length; i++) {
            residualsVoltageVelocityWise[i]
                            = voltageDataArray[i] - regression.predict(velocityDataArray[i]);
        }
        PolynomialRegression accelerationRegression
                        = new PolynomialRegression(accelerationDataArray,
                                                   residualsVoltageVelocityWise,
                                                   2);
        System.out.println("FF Characterization Results:");
        System.out.println("\tCount=" + Integer.toString(velocityData.size())
                           + "");
        System.out.println(String.format("\tR2=%.5f", regression.R2(velocityDataArray, voltageDataArray)));
        System.out.println(String.format("\tkS=%.5f", regression.beta(0)));
        System.out.println(String.format("\tkV=%.5f", regression.beta(1)));
        System.out.println(String.format("\tkA=%.5f", accelerationRegression.beta(1)));
    }
}

class PolynomialRegression {
    private SimpleMatrix coefficients;

    public PolynomialRegression(double[] x, double[] y, int degree) {
        // Add a column of ones to the x matrix for the y-intercept
        SimpleMatrix xMatrix = new SimpleMatrix(x.length, degree + 1);
        for (int i = 0; i < x.length; i++) {
            xMatrix.set(i, 0, 1);
            for (int j = 1; j <= degree; j++) {
                xMatrix.set(i, j, Math.pow(x[i], j));
            }
        }
        // Use the Moore-Penrose pseudoinverse to find the coefficients
        SimpleMatrix yMatrix = new SimpleMatrix(y.length, 1, true, y);
        SimpleMatrix transpose = xMatrix.transpose();
        SimpleMatrix xTx = transpose.mult(xMatrix);
        SimpleMatrix xTxInv = xTx.invert();
        SimpleMatrix xTy = transpose.mult(yMatrix);
        coefficients = xTxInv.mult(xTy);
    }

    public double predict(double x) {
        double y = 0;
        for (int i = 0; i < coefficients.numRows(); i++) {
            y += coefficients.get(i) * Math.pow(x, i);
        }
        return y;
    }

    public double R2(double[] x, double[] y) {
        double sst = 0, sse = 0, yMean = 0;
        for (int i = 0; i < y.length; i++) {
            yMean += y[i];
        }
        yMean /= y.length;
        for (int i = 0; i < y.length; i++) {
            double yPred = predict(x[i]);
            sst += (y[i] - yMean) * (y[i] - yMean);
            sse += (y[i] - yPred) * (y[i] - yPred);
        }
        return 1 - sse / sst;
    }

    public double beta(int degree) {
        return coefficients.get(degree);
    }
}