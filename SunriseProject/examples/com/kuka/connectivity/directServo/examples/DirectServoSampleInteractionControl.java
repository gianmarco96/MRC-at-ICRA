package com.kuka.connectivity.directServo.examples;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.common.StatisticTimer;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.motionModel.directServo.DirectServo;
import com.kuka.connectivity.motionModel.directServo.IDirectServoRuntime;
import com.kuka.connectivity.motionModel.smartServo.ServoMotion;
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
 * This example activates a DirectServo motion in Cartesian impedance control mode, sends a sequence of Cartesian set
 * points and modifies compliance parameters during the motion.
 * 
 */
public class DirectServoSampleInteractionControl extends RoboticsAPIApplication
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
                    .info("DirectServo motion will be available for position controlled mode only, until validation is performed");
        }
    }

    /**
     * Creates a smartServo motion with the given control mode and moves around.
     * 
     * @param controlMode
     *            the control mode which shall be used
     * @see {@link CartesianImpedanceControlMode}
     */
    public void runDirectServoMotion(final IMotionControlMode controlMode)
    {
        final JointPosition initialPosition = new JointPosition(
                _lbr.getCurrentJointPosition());

        final DirectServo aDirectServoMotion = new DirectServo(initialPosition);

        aDirectServoMotion.setMinimumTrajectoryExecutionTime(40e-3);

        getLogger().info("Starting the DirectServo motion in " + controlMode);
        _toolAttachedToLBR.getDefaultMotionFrame().moveAsync(
                aDirectServoMotion.setMode(controlMode));

        final IDirectServoRuntime theServoRuntime = aDirectServoMotion
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
            // freqency * ...
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

                theServoRuntime.updateWithRealtimeSystem();

                final double curTime = System.nanoTime() - startTimeStamp;
                final double sinArgument = omega * curTime;

                for (int k = 0; k < destination.getAxisCount(); ++k)
                {
                    destination.set(k, Math.sin(sinArgument)
                            * AMPLITUDE + initialPosition.get(k));
                }
                theServoRuntime
                        .setDestination(destination);

                // Modify the stiffness settings every now and then               
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
                        theServoRuntime
                                .changeControlModeSettings(cartImp);
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

        // Print statistics and parameters of the motion
        getLogger().info("Displaying final states after loop "
                + controlMode.getClass().getName());

        getLogger().info(getClass().getName() + "\n" + theServoRuntime.toString());

        // Stop the motion
        theServoRuntime.stopMotion();
        getLogger().info("Statistic Timing of Overall Loop " + timing);
        if (timing.getMeanTimeMillis() > 150)
        {
            getLogger().info("Statistic Timing is unexpected slow, you should try to optimize TCP/IP Transfer");
            getLogger().info("You should check the TCP/IP Stack Configuration - see the manual for details");
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

    /** Sample to switch the motion control mode */
    protected void switchMotionControlMode()
    {
        getLogger().info("Switch Motion Control Mode Sample");
        final boolean debugPrintoutFlag = false;
        /* Prepare two control modes for the sample */
        final CartesianImpedanceControlMode cartImp = createCartImp();

        final JointPosition initialPosition = new JointPosition(
                _lbr.getCurrentJointPosition());

        final DirectServo firstDirectServoMotion = new DirectServo(initialPosition);

        firstDirectServoMotion.setMode(cartImp);

        // Set the control mode as member of the realtime motion
        _toolAttachedToLBR.getDefaultMotionFrame().moveAsync(firstDirectServoMotion);

        // Fetch the Runtime of the Motion part
        // NOTE: the Runtime will exist AFTER motion command was issued
        final IDirectServoRuntime theFirstRuntime = firstDirectServoMotion
                .getRuntime();

        /* Do Interaction with that mode Run set points etc... */
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
        for (int k = 0; k < initialPosition.getAxisCount(); k++)
        {
            initialPosition.set(k, initialPosition.get(k) + 0.01);
        }

        final DirectServo secondDirectServoMotion = new DirectServo(initialPosition);
        secondDirectServoMotion.setJointVelocityRel(0.1);

        // / Activate the Motion -- it will become truely active, as the first
        // one vanishes      
        _toolAttachedToLBR.getDefaultMotionFrame().moveAsync(secondDirectServoMotion);

        getLogger().info("Now blending over to -> Sending Stop Request "
                );

        // ///////////////////////////////////
        // Now blend over - stop the first,
        // the second will immediately take over
        theFirstRuntime.stopMotion();
        // get the runtime of the second motion

        final IDirectServoRuntime theSecondRuntime = secondDirectServoMotion
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

        runDirectServoMotion(cartImp);

        // Return to initial position
        moveToInitialPosition();

        final PositionControlMode positionCtrlMode = new PositionControlMode();

        runDirectServoMotion(positionCtrlMode);

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
        final DirectServoSampleInteractionControl app = new DirectServoSampleInteractionControl();

        app.runApplication();

    }
}
