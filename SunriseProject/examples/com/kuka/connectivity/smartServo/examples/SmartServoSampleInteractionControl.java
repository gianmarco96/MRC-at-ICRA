package com.kuka.connectivity.smartServo.examples;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.common.StatisticTimer;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.motionModel.smartServo.ISmartServoRuntime;
import com.kuka.connectivity.motionModel.smartServo.ServoMotion;
import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;

/**
 * This example activates a SmartServo motion in Cartesian impedance control mode, sends a sequence of Cartesian set
 * points and modifies compliance parameters during the motion.
 * 
 */
public class SmartServoSampleInteractionControl extends RoboticsAPIApplication
{
    private LBR _lbr;
    private Tool _toolAttachedToLBR;
    private LoadData _loadData;

    // Tool Data
    private static final String TOOL_FRAME = "toolFrame";
    private static final double[] TRANSLATION_OF_TOOL = { 0, 0, 100 };
    private static final double MASS = 0;
    private static final double[] CENTER_OF_MASS_IN_MILLIMETER = { 0, 0, 100 };

    private static final int NUM_RUNS = 600;
    private static final double AMPLITUDE = 0.2;
    private static final double FREQENCY = 0.1;

    private static final int MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT = 30;

    @Override
    public void initialize()
    {
        _lbr = getContext().getDeviceFromType(LBR.class);

        // Create a Tool by Hand this is the tool we want to move with some mass
        // properties and a TCP-Z-offset of 100.
        _loadData = new LoadData();
        _loadData.setMass(MASS);
        _loadData.setCenterOfMass(
                CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
                CENTER_OF_MASS_IN_MILLIMETER[2]);
        _toolAttachedToLBR = new Tool("Tool", _loadData);

        XyzAbcTransformation trans = XyzAbcTransformation.ofTranslation(
                TRANSLATION_OF_TOOL[0], TRANSLATION_OF_TOOL[1],
                TRANSLATION_OF_TOOL[2]);
        ObjectFrame aTransformation = _toolAttachedToLBR.addChildFrame(TOOL_FRAME
                + "(TCP)", trans);
        _toolAttachedToLBR.setDefaultMotionFrame(aTransformation);
        // Attach tool to the robot
        _toolAttachedToLBR.attachTo(_lbr.getFlange());
    }

    /**
     * Move to an initial Position WARNING: MAKE SURE, THAT the pose is collision free.
     */
    public void moveToInitialPosition()
    {
        _lbr.move(ptp(0., Math.PI / 180 * 30., 0., -Math.PI / 180 * 60., 0.,
                Math.PI / 180 * 90., 0.).setJointVelocityRel(0.1));
        /* Note: The Validation itself justifies, that in this very time instance, the load parameter setting was
         * sufficient. This does not mean by far, that the parameter setting is valid in the sequel or lifetime of this
         * program */
        if (!ServoMotion.validateForImpedanceMode(_lbr))
        {
            getLogger()
                    .info("Validation of torque model failed - correct your mass property settings");
            getLogger()
                    .info("Servo motion will be available for position controlled mode only, until validation is performed");
        }
    }

    /**
     * Creates a smartServo motion with the given control mode and moves around.
     * 
     * @param controlMode
     *            the control mode which shall be used
     * @see {@link CartesianImpedanceControlMode}
     */

    protected void runSmartServoMotion(final IMotionControlMode controlMode)
    {
        final boolean doDebugPrints = false;

        final JointPosition initialPosition = new JointPosition(
                _lbr.getCurrentJointPosition());

        final SmartServo aSmartServoMotion = new SmartServo(initialPosition);

        // Set the motion properties to 10% of the systems abilities
        aSmartServoMotion.setJointAccelerationRel(0.1);
        aSmartServoMotion.setJointVelocityRel(0.1);

        aSmartServoMotion.setMinimumTrajectoryExecutionTime(20e-3);

        getLogger().info("Starting the SmartServo in " + controlMode);
        _toolAttachedToLBR.getDefaultMotionFrame().moveAsync(
                aSmartServoMotion.setMode(controlMode));

        // Fetch the Runtime of the Motion part
        final ISmartServoRuntime theSmartServoRuntime = aSmartServoMotion
                .getRuntime();

        // create an JointPosition Instance, to play with
        final JointPosition destination = new JointPosition(
                _lbr.getJointCount());

        // For Roundtrip time measurement...
        final StatisticTimer timing = new StatisticTimer();
        try
        {
            // do a cyclic loop
            // Refer to some timing...
            // in nanosec
            final double omega = FREQENCY * 2 * Math.PI * 1e-9;
            final long startTimeStamp = System.nanoTime();

            for (int i = 0; i < NUM_RUNS; ++i)
            {
                // Timing - draw one step
                final OneTimeStep aStep = timing.newTimeStep();
                // ///////////////////////////////////////////////////////
                // Insert your code here
                // e.g Visual Servoing or the like
                // emulate some computational effort - or waiting for external
                // stuff
                ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);

                theSmartServoRuntime.updateWithRealtimeSystem();

                // Get the measured position in cartesian...
                final JointPosition curMsrJntPose = theSmartServoRuntime
                        .getAxisQMsrOnController();

                final double curTime = System.nanoTime() - startTimeStamp;
                final double sinArgument = omega * curTime;

                for (int k = 0; k < destination.getAxisCount(); ++k)
                {
                    destination.set(k, Math.sin(sinArgument)
                            * AMPLITUDE + initialPosition.get(k));
                }
                theSmartServoRuntime
                        .setDestination(destination);

                //
                // Modify the stiffness settings every now and then
                //
                if (i % (NUM_RUNS / 10) == 0)
                {

                    // update realtime system
                    if (controlMode instanceof CartesianImpedanceControlMode)
                    {
                        final CartesianImpedanceControlMode cartImp = (CartesianImpedanceControlMode) controlMode;
                        final double aTransStiffVal = Math.max(100. * (i
                                / (double) NUM_RUNS + 1), 1000.);
                        final double aRotStiffVal = Math.max(10. * (i
                                / (double) NUM_RUNS + 1), 150.);
                        cartImp.parametrize(CartDOF.TRANSL).setStiffness(aTransStiffVal);
                        cartImp.parametrize(CartDOF.ROT).setStiffness(aRotStiffVal);
                        // Send the new Stiffness settings down to the
                        // controller
                        theSmartServoRuntime
                                .changeControlModeSettings(cartImp);
                    }
                }

                if (doDebugPrints)
                {

                    getLogger().info("Step " + i + " New Goal "
                            + destination);
                    getLogger().info("LBR Position "
                            + _lbr.getCurrentJointPosition());
                    getLogger().info("Measured LBR position "
                            + curMsrJntPose);
                    if (i % 100 == 0)
                    {
                        getLogger().info("Simple Joint Test - step " + i + "\n" + theSmartServoRuntime.toString());
                    }
                }
                aStep.end();
            }
        }
        catch (final Exception e)
        {
            getLogger().info(e.getLocalizedMessage());
            e.printStackTrace();
        }

        //Print statistics and parameters of the motion
        getLogger().info("Displaying final states after loop "
                + controlMode.getClass().getName());
        getLogger().info(getClass().getName() + theSmartServoRuntime.toString());
        // Stop the motion
        theSmartServoRuntime.stopMotion();
        getLogger().info("Statistic Timing of Overall Loop " + timing);
        if (timing.getMeanTimeMillis() > 150)
        {
            getLogger().info("Statistic Timing is unexpected slow, you should try to optimize TCP/IP Transfer");
            getLogger().info("Under Windows, you should play with the registry, see the e.g. the RealtimePTP Class javaDoc for details");
        }

    }

    /**
     * Create the CartesianImpedanceControlMode class for motion parameterisation.
     * 
     * @see {@link CartesianImpedanceControlMode}
     * @return the created control mode
     */
    protected CartesianImpedanceControlMode createCartImp()
    {
        final CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();
        cartImp.parametrize(CartDOF.TRANSL).setStiffness(1000.0);
        cartImp.parametrize(CartDOF.ROT).setStiffness(100.0);
        cartImp.setNullSpaceStiffness(100.);
        // For your own safety, shrink the motion abilities to useful limits
        cartImp.setMaxPathDeviation(50., 50., 50., 50., 50., 50.);
        return cartImp;
    }

    /** Sample to switch the motion control mode. */
    protected void switchMotionControlMode()
    {
        getLogger().info("Switch Motion Control Mode Sample");
        final boolean debugPrintoutFlag = false;
        /* Prepare two control modes for the sample */
        final CartesianImpedanceControlMode cartImp = createCartImp();

        final JointPosition initialPosition = new JointPosition(
                _lbr.getCurrentJointPosition());

        final SmartServo firstSmartServoMotion = new SmartServo(initialPosition);

        // Set the motion properties to 10% of the systems abilities
        firstSmartServoMotion.setJointAccelerationRel(0.1);
        firstSmartServoMotion.setJointVelocityRel(0.1);
        firstSmartServoMotion.setMode(cartImp);

        getLogger().info("Starting the SmartServo in " + cartImp);
        _toolAttachedToLBR.getDefaultMotionFrame().moveAsync(firstSmartServoMotion);

        // Fetch the Runtime of the Motion part
        // NOTE: the Runtime will exist AFTER motion command was issued
        final ISmartServoRuntime theFirstRuntime = firstSmartServoMotion
                .getRuntime();

        /* Do Interaction with that mode Run set points etc... */
        initialPosition.set(2, -1);
        theFirstRuntime.setDestination(initialPosition);

        /* Here: Just wait, until fine interpolation has finished */
        while (!theFirstRuntime.isDestinationReached())
        {
            ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);
            theFirstRuntime.updateWithRealtimeSystem();
            if (debugPrintoutFlag)
            {
                getLogger().info("Waiting for reaching goal "
                        + theFirstRuntime);
            }
        }

        // Open second Motion
        for (int i = 0; i < initialPosition.getAxisCount(); i++)
        {
            initialPosition.set(i, initialPosition.get(i) + 0.1);
        }

        final SmartServo secondSmartServoMotion = new SmartServo(initialPosition);

        secondSmartServoMotion.setJointAccelerationRel(0.2);
        secondSmartServoMotion.setJointVelocityRel(0.1);

        // / Activate the Motion -- it will become truely active, as the first
        // one vanishes
        // Set the control mode as member of the realtime motion
        _toolAttachedToLBR.getDefaultMotionFrame().moveAsync(secondSmartServoMotion);

        getLogger().info("Now blending over to -> Sending Stop Request "
                );

        // ///////////////////////////////////
        // Now blend over - stop the first,
        // the second will immediately take over
        theFirstRuntime.stopMotion();

        final ISmartServoRuntime theSecondRuntime = secondSmartServoMotion
                .getRuntime();
        theSecondRuntime.setDestination(initialPosition);
        /* do further computations... */

        /* Here: Just wait, until fine interpolation has finished */
        while (!theSecondRuntime.isDestinationReached())
        {
            ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);
            theSecondRuntime.updateWithRealtimeSystem();
            if (debugPrintoutFlag)
            {
                getLogger().info("Waiting for reaching goal Runtime 2 "
                        + theSecondRuntime);
            }
        }

        theSecondRuntime.stopMotion();

        getLogger().info("Result of Motion 1 " + theFirstRuntime);
        getLogger().info("Result of Motion 2 " + theSecondRuntime);
    }

    @Override
    public void run()
    {

        moveToInitialPosition();

        // Initialize Cartesian impedance mode       
        final CartesianImpedanceControlMode cartImp = createCartImp();

        runSmartServoMotion(cartImp);

        // Return to initial position
        moveToInitialPosition();

        final PositionControlMode positionCtrlMode = new PositionControlMode();

        runSmartServoMotion(positionCtrlMode);

        moveToInitialPosition();

        switchMotionControlMode();

    }

    /**
     * Main routine, which starts the application.
     * 
     * @param args
     *            arguments
     */
    public static void main(final String[] args)
    {
        final SmartServoSampleInteractionControl app = new SmartServoSampleInteractionControl();
        app.runApplication();
    }
}
