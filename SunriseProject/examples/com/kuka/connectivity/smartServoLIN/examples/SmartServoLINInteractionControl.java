package com.kuka.connectivity.smartServoLIN.examples;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.common.StatisticTimer;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.motionModel.smartServo.ServoMotion;
import com.kuka.connectivity.motionModel.smartServoLIN.ISmartServoLINRuntime;
import com.kuka.connectivity.motionModel.smartServoLIN.SmartServoLIN;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;

/**
 * This example activates a SmartServoLIN motion in Cartesian impedance control mode, sends a sequence of Cartesian set
 * points, describing a sine function in z-direction and modifies compliance parameters during the motion.
 * 
 */
public class SmartServoLINInteractionControl extends RoboticsAPIApplication
{

    private LBR _lbr;
    private Tool _toolAttachedToLBR;
    private LoadData _loadData;
    private ISmartServoLINRuntime _theSmartServoLINRuntime = null;

    // Tool Data
    private static final String TOOL_FRAME = "toolFrame";
    private static final double[] TRANSLATION_OF_TOOL = { 0, 0, 100 };
    private static final double MASS = 0;
    private static final double[] CENTER_OF_MASS_IN_MILLIMETER = { 0, 0, 100 };

    private static final int NUM_RUNS = 600;
    private static final double AMPLITUDE = 70;
    private static final double FREQENCY = 0.6;

    private static final double[] MAX_TRANSLATION_VELOCITY = { 150, 150, 150 };
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

    @Override
    public void run()
    {
        getLogger().info("Move to start position.");
        moveToInitialPosition();

        // Initialize Cartesian impedance control mode
        final CartesianImpedanceControlMode cartImp = createCartImp();

        getLogger()
                .info("Sample Application - SmartServoLIN motion in cartesian impedance control mode");
        runSmartServoLINMotion(cartImp);

        // Return to initial position
        moveToInitialPosition();

        // Initialize position control mode
        final PositionControlMode positionCtrlMode = new PositionControlMode();

        getLogger()
                .info("Sample Application -  SmartServoLIN motion in position control mode");
        runSmartServoLINMotion(positionCtrlMode);
    }

    /**
     * Creates a smartServoLIN Motion with the given control mode and moves around.
     * 
     * @param controlMode
     *            the control mode which shall be used
     * @see {@link CartesianImpedanceControlMode}
     */

    protected void runSmartServoLINMotion(final IMotionControlMode controlMode)
    {
        AbstractFrame initialPosition = _lbr.getCurrentCartesianPosition(_lbr
                .getFlange());

        // Create a new smart servo linear motion
        SmartServoLIN aSmartServoLINMotion = new SmartServoLIN(initialPosition);

        aSmartServoLINMotion.setMaxTranslationVelocity(MAX_TRANSLATION_VELOCITY);
        aSmartServoLINMotion.setMinimumTrajectoryExecutionTime(20e-3);

        getLogger().info("Starting the SmartServoLIN in " + controlMode);
        _lbr.moveAsync(aSmartServoLINMotion.setMode(controlMode));

        getLogger().info("Get the runtime of the SmartServoLIN motion");
        _theSmartServoLINRuntime = aSmartServoLINMotion.getRuntime();

        StatisticTimer timing = new StatisticTimer();

        getLogger().info("Do sine movement");
        timing = startSineMovement(_theSmartServoLINRuntime, timing, controlMode);

        ThreadUtil.milliSleep(500);

        // Print statistic timing
        getLogger().info(
                getClass().getName() + _theSmartServoLINRuntime.toString());

        getLogger().info("Stop the SmartServoLIN motion");
        _theSmartServoLINRuntime.stopMotion();

        // Statistic Timing of sine movement loop
        if (timing.getMeanTimeMillis() > 150)
        {
            getLogger()
                    .info("Statistic Timing is unexpected slow, you should try to optimize TCP/IP Transfer");
            getLogger()
                    .info("Under Windows, you should play with the registry, see the e.g. user manual");
        }
    }

    /**
     * Create the CartesianImpedanceControlMode class for motion parameterization.
     * 
     * @see {@link CartesianImpedanceControlMode}
     * @return the created control mode
     */
    private CartesianImpedanceControlMode createCartImp()
    {
        final CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();
        cartImp.parametrize(CartDOF.Z).setStiffness(800.0);
        return cartImp;
    }

    private StatisticTimer startSineMovement(
            ISmartServoLINRuntime theSmartServoLINRuntime,
            StatisticTimer timing, IMotionControlMode mode)
    {
        Frame aFrame = theSmartServoLINRuntime
                .getCurrentCartesianDestination(_lbr.getFlange());

        try
        {
            getLogger().info("Start SmartServoLIN sine movement");
            double omega = FREQENCY * 2 * Math.PI * 1e-9;
            long startTimeStamp = System.nanoTime();
            int i;

            for (i = 0; i < NUM_RUNS; ++i)
            {
                final OneTimeStep aStep = timing.newTimeStep();
                // ///////////////////////////////////////////////////////
                // Insert your code here
                // e.g Visual Servoing or the like
                // Synchronize with the realtime system

                ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);

                // Update the smart servo LIN runtime
                theSmartServoLINRuntime.updateWithRealtimeSystem();

                double curTime = System.nanoTime() - startTimeStamp;
                double sinArgument = omega * curTime;

                // Compute the sine function
                Frame destFrame = new Frame(aFrame);
                destFrame.setZ(AMPLITUDE * Math.sin(sinArgument));

                // Set new destination
                theSmartServoLINRuntime.setDestination(destFrame);
                aStep.end();
            }

            // Modify the stiffness settings every now and then
            if (i % (NUM_RUNS / 10) == 0)
            {
                if (mode instanceof CartesianImpedanceControlMode)
                {
                    // We are in CartImp Mode,
                    // Modify the settings:
                    // NOTE: YOU HAVE TO REMAIN POSITIVE SEMI-DEFINITE !!
                    // NOTE: DONT CHANGE TOO FAST THE SETTINGS, ELSE YOU
                    // WILL DESTABILIZE THE CONTROLLER
                    final CartesianImpedanceControlMode cartImp = (CartesianImpedanceControlMode) mode;
                    final double aTransStiffVal = Math.max(100. * (i
                            / (double) NUM_RUNS + 1), 1000.);
                    final double aRotStiffVal = Math.max(10. * (i
                            / (double) NUM_RUNS + 1), 150.);
                    cartImp.parametrize(CartDOF.TRANSL).setStiffness(
                            aTransStiffVal);
                    cartImp.parametrize(CartDOF.ROT).setStiffness(aRotStiffVal);

                    // Send the new Stiffness settings down to the
                    // controller
                    theSmartServoLINRuntime.changeControlModeSettings(cartImp);
                }
            }
        }
        catch (Exception e)
        {
            getLogger().error(e.getLocalizedMessage());
            e.printStackTrace();
        }
        return timing;
    }

    /**
     * Main routine, which starts the application.
     * 
     * @param args
     *            arguments
     */
    public static void main(final String[] args)
    {
        final SmartServoLINInteractionControl app = new SmartServoLINInteractionControl();
        app.runApplication();
    }
}
