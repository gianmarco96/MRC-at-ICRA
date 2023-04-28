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
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;

/**
 * This example activates a DirectServo motion in position control mode, sends a sequence of joint specific set points,
 * describing a sine function and evaluates the statistic timing.
 */
public class DirectServoSampleSimpleJointMotion extends RoboticsAPIApplication
{
    private LBR _lbr;
    private IDirectServoRuntime _theDirectServoRuntime = null;

    // Tool Data
    private Tool _toolAttachedToLBR;
    private LoadData _loadData;

    // Tool Data
    private static final String TOOL_FRAME = "toolFrame";
    private static final double[] TRANSLATION_OF_TOOL = { 0, 0, 100 };
    private static final double MASS = 0;
    private static final double[] CENTER_OF_MASS_IN_MILLIMETER = { 0, 0, 100 };

    private static final int MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT = 30;
    private static final int NUM_RUNS = 1000;
    private static final double AMPLITUDE = 0.2;
    private static final double FREQENCY = 0.1;
    private int _steps = 0;

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
     * Move to an initial Position WARNING: MAKE SURE, THAT the pose is collision free
     */
    public void moveToInitialPosition()
    {
        _toolAttachedToLBR.move(
                ptp(0., Math.PI / 180 * 30., 0., -Math.PI / 180 * 60., 0.,
                        Math.PI / 180 * 90., 0.).setJointVelocityRel(0.1));
        /* Note: The Validation itself justifies, that in this very time instance, the load parameter setting was
         * sufficient. This does not mean by far, that the parameter setting is valid in the sequel or lifetime of this
         * program */
        try
        {
            if (!ServoMotion.validateForImpedanceMode(_toolAttachedToLBR))
            {
                getLogger().info("Validation of torque model failed - correct your mass property settings");
                getLogger().info("DirectServo will be available for position controlled mode only, until validation is performed");
            }
        }
        catch (IllegalStateException e)
        {
            getLogger().info("Omitting validation failure for this sample\n"
                    + e.getMessage());
        }
    }

    @Override
    public void run()
    {
        moveToInitialPosition();

        JointPosition initialPosition = new JointPosition(
                _lbr.getCurrentJointPosition());
        DirectServo aDirectServoMotion = new DirectServo(initialPosition);

        aDirectServoMotion.setMinimumTrajectoryExecutionTime(40e-3);

        getLogger().info("Starting DirectServo motion in position control mode");
        _toolAttachedToLBR.getDefaultMotionFrame().moveAsync(aDirectServoMotion);

        // Fetch the Runtime of the Motion part
        // NOTE: the Runtime exists AFTER motion command was issued
        _theDirectServoRuntime = aDirectServoMotion
                .getRuntime();

        // create an JointPosition Instance, to play with
        JointPosition destination = new JointPosition(
                _lbr.getJointCount());

        getLogger().info("Start loop");
        // For Roundtrip time measurement...
        StatisticTimer timing = new StatisticTimer();
        try
        {
            // do a cyclic loop
            // Refer to some timing...
            // in nanosec
            double omega = FREQENCY * 2 * Math.PI * 1e-9;
            long startTimeStamp = System.nanoTime();

            for (_steps = 0; _steps < NUM_RUNS; ++_steps)
            {
                // Timing - draw one step
                OneTimeStep aStep = timing.newTimeStep();
                // ///////////////////////////////////////////////////////
                // Insert your code here
                // e.g Visual Servoing or the like
                // emulate some computational effort - or waiting for external
                // stuff
                ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);
                _theDirectServoRuntime.updateWithRealtimeSystem();

                double curTime = System.nanoTime() - startTimeStamp;
                double sinArgument = omega * curTime;

                for (int k = 0; k < destination.getAxisCount(); ++k)
                {
                    destination.set(k, Math.sin(sinArgument)
                            * AMPLITUDE + initialPosition.get(k));
                    if (k > 5)
                    {
                        destination.set(k, initialPosition.get(k));
                    }
                }
                _theDirectServoRuntime
                        .setDestination(destination);

                aStep.end();
            }
        }
        catch (Exception e)
        {
            getLogger().info(e.getLocalizedMessage());
            e.printStackTrace();
        }
        ThreadUtil.milliSleep(1000);
        //Print statistics and parameters of the motion
        getLogger().info("Displaying final states after loop ");
        getLogger().info(getClass().getName() + " \n" + _theDirectServoRuntime.toString());
        // Stop the motion
        getLogger().info("Stop the SmartServo motion");
        _theDirectServoRuntime.stopMotion();
        getLogger().info("Statistic Timing of Overall Loop " + timing);
        if (timing.getMeanTimeMillis() > 150)
        {
            getLogger().info("Statistic Timing is unexpected slow, you should try to optimize TCP/IP Transfer");
            getLogger().info("Under Windows, you should play with the registry, see the e.g. the SmartServo Class javaDoc for details");
        }
    }

    /**
     * Main routine, which starts the application
     */
    public static void main(String[] args)
    {
        DirectServoSampleSimpleJointMotion app = new DirectServoSampleSimpleJointMotion();
        app.runApplication();

    }
}
