package com.kuka.connectivity.smartServo.examples;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.motionModel.smartServo.ISmartServoRuntime;
import com.kuka.connectivity.motionModel.smartServo.ServoMotion;
import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;

/**
 * This example activates a SmartServo motion in position control mode, sends a sequence of Cartesian set points,
 * describing a sine function and evaluates the statistic timing.
 */
public class SmartServoSampleSimpleCartesian extends RoboticsAPIApplication
{

    private LBR _lbr;

    // Tool Data
    private Tool _toolAttachedToLBR;
    private LoadData _loadData;
    private static final String TOOL_FRAME = "toolFrame";
    private static final double[] TRANSLATION_OF_TOOL = { 0, 0, 100 };
    private static final double MASS = 0;
    private static final double[] CENTER_OF_MASS_IN_MILLIMETER = { 0, 0, 100 };

    private static final int MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT = 30;
    private static final int NUM_RUNS = 1000;
    private static final double AMPLITUDE = 70;
    private static final double FREQENCY = 0.6;

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
    private void moveToInitialPosition()
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
                getLogger().info("SmartServo will be available for position controlled mode only, until validation is performed");
            }
        }
        catch (IllegalStateException e)
        {
            getLogger().info("Omitting validation failure for this sample\n"
                    + e.getMessage());
        }
    }

    /**
     * Main Application Routine.
     */
    @Override
    public void run()
    {
        moveToInitialPosition();

        boolean doDebugPrints = false;

        SmartServo aSmartServoMotion = new SmartServo(
                _lbr.getCurrentJointPosition());

        aSmartServoMotion.useTrace(true);

        aSmartServoMotion.setMinimumTrajectoryExecutionTime(5e-3);

        getLogger().info("Starting SmartServo motion in position control mode");
        _toolAttachedToLBR.moveAsync(aSmartServoMotion);

        getLogger().info("Get the runtime of the SmartServo motion");
        ISmartServoRuntime theServoRuntime = aSmartServoMotion
                .getRuntime();

        Frame aFrame = theServoRuntime.getCurrentCartesianDestination(_toolAttachedToLBR.getDefaultMotionFrame());

        try
        {
            // do a cyclic loop
            // Do some timing...
            // in nanosec
            double omega = FREQENCY * 2 * Math.PI * 1e-9;
            long startTimeStamp = System.nanoTime();
            for (int i = 0; i < NUM_RUNS; ++i)
            {
                // Insert your code here
                // e.g Visual Servoing or the like

                // Synchronize with the realtime system
                theServoRuntime.updateWithRealtimeSystem();

                // Get the measured position 
                Frame msrPose = theServoRuntime
                        .getCurrentCartesianDestination(_toolAttachedToLBR.getDefaultMotionFrame());

                if (doDebugPrints)
                {
                    getLogger().info("Current cartesian goal " + aFrame);
                    getLogger().info("Current joint destination "
                            + theServoRuntime.getCurrentJointDestination());
                }

                // Do some Computation
                // emulate some computational effort - or waiting for external
                // stuff
                ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);

                // do a cyclic loop
                double curTime = System.nanoTime() - startTimeStamp;
                double sinArgument = omega * curTime;

                // compute a new commanded position
                Frame destFrame = aFrame.copyWithRedundancy();
                double offset = AMPLITUDE * Math.sin(sinArgument);
                destFrame.setZ(destFrame.getZ() + offset);

                if (doDebugPrints)
                {
                    getLogger().info("New cartesian goal " + destFrame);
                    getLogger().info("LBR position "
                            + _lbr.getCurrentCartesianPosition(_lbr
                                    .getFlange()));
                    getLogger().info("Measured cartesian pose from runtime "
                            + msrPose);

                    if ((i % 100) == 0)
                    {
                        // Some internal values, which can be displayed
                        getLogger().info("Simple cartesian test " + theServoRuntime.toString());
                    }
                }

                theServoRuntime.setDestination(destFrame);
            }
        }
        catch (Exception e)
        {
            getLogger().error(e.toString());
            e.printStackTrace();
        }

        //Print statistics and parameters of the motion
        getLogger().info("Simple cartesian test " + theServoRuntime.toString());

        getLogger().info("Stop the SmartServo motion");
        theServoRuntime.stopMotion();
    }

    /**
     * Main routine, which starts the application.
     * 
     * @param args
     *            arguments
     */
    public static void main(String[] args)
    {
        SmartServoSampleSimpleCartesian app = new SmartServoSampleSimpleCartesian();
        app.runApplication();
    }
}
