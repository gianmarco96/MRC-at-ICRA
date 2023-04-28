package lbrExampleApplications;


import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.LBRE1Redundancy;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.LIN;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

public class Motions extends RoboticsAPIApplication {
	private LBR lbr;

	final static double radiusOfCircMove=120;
	final static int nullSpaceAngle = 80;

	final static double offsetAxis2And4=Math.toRadians(20);
	final static double offsetAxis4And6=Math.toRadians(-40);
	double[] loopCenterPosition= new double[]{
			0, offsetAxis2And4, 0, offsetAxis2And4 +offsetAxis4And6 -Math.toRadians(90), 0, offsetAxis4And6,Math.toRadians(90)};


	private final static String informationText=
			"This application is intended for floor mounted robots!"+ "\n" +
			"\n" +
			"The robot moves to the start position and based on this position, a motion that " +
			"describes the symbol of lemniscate (a 'horizontal eight') will be executed." + "\n" +
			"In a next step the robot will move in nullspace by "+nullSpaceAngle+"° in both directions.";

	public void initialize() {		
		lbr = getContext().getDeviceFromType(LBR.class);
	}

	public void run() {		
		getLogger().info("Show modal dialog and wait for user to confirm");
        int isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, informationText, "OK", "Cancel");
        if (isCancel == 1)
        {
            return;
        }

		getLogger().info("Move to start position of the lemniscate motion");	
		PTP ptpToLoopCenter = ptp(loopCenterPosition);
		ptpToLoopCenter.setJointVelocityRel(0.25);
		lbr.move(ptpToLoopCenter);

		getLogger().info("Compute spline for lemniscate motion");	
		Frame startFrame = lbr.getCurrentCartesianPosition(lbr.getFlange());
		Spline lemniscateSpline = createLemniscateSpline(startFrame).setJointJerkRel(0.5).setCartVelocity(250);

		getLogger().info("Execute lemniscate motion");
		lemniscateSpline.setJointVelocityRel(0.25);
		lbr.move(lemniscateSpline);

		getLogger().info("Move in nullspace -"+nullSpaceAngle+"°");		
		Frame centerFrameWithChangedE1_1 = createChildFrameAndSetE1Offset(startFrame,Math.toRadians(-nullSpaceAngle));
		LIN linToCenterFrameWithE1_1 = lin(centerFrameWithChangedE1_1);
		linToCenterFrameWithE1_1.setJointVelocityRel(0.25);
		lbr.move(linToCenterFrameWithE1_1);

		getLogger().info("Move in nullspace "+nullSpaceAngle+"°");
		Frame centerFrameWithChangedE1_2 = createChildFrameAndSetE1Offset(startFrame,Math.toRadians(nullSpaceAngle));
		LIN linToCenterFrameWithE1_2 = lin(centerFrameWithChangedE1_2);
		linToCenterFrameWithE1_2.setJointVelocityRel(0.25);
		lbr.move(linToCenterFrameWithE1_2);
		
		getLogger().info("Move to start position");
		LIN linToStartFrame = lin(startFrame);
		linToStartFrame.setJointVelocityRel(0.25);
		lbr.move(linToStartFrame);
	}

	

	private Spline createLemniscateSpline(Frame centerFrame) {

		// Create a new frame with the center frame as parent. Set an offset for the x axis to this parent.
		Frame rightFrame=(new Frame(centerFrame)).setX(2*radiusOfCircMove);

		// Create a new frame with the center frame as parent. Set an offset for the x axis to this parent.
		Frame leftFrame= (new Frame(centerFrame)).setX(-2*radiusOfCircMove);	

		// Create a new frame with the center frame as parent. Set an offset for the x and y axes to this parent.
		Frame topLeftFrame= (new Frame(centerFrame)).setX(-radiusOfCircMove).setY(radiusOfCircMove);		

		// Create a new frame with the center frame as parent. Set an offset for the x and y axes to this parent.
		Frame topRightFrame= (new Frame(centerFrame)).setX(+radiusOfCircMove).setY(radiusOfCircMove);		

		// Create a new frame with the center frame as parent. Set an offset for the x and y axes to this parent.
		Frame bottomRightFrame= (new Frame(centerFrame)).setX(+radiusOfCircMove).setY(-radiusOfCircMove);

		// Create a new frame with the center frame as parent. Set an offset for the x and y axes to this parent.
		Frame bottomLeftFrame= (new Frame(centerFrame)).setX(-radiusOfCircMove).setY(-radiusOfCircMove);

		// Create a spline that describes a lemniscate
		Spline spline = new Spline(
				spl(bottomLeftFrame),
				spl(leftFrame),
				spl(topLeftFrame),
				spl(centerFrame),
				spl(bottomRightFrame),
				spl(rightFrame),
				spl(topRightFrame),
				spl(centerFrame));
		return spline;
	}

	
	private Frame createChildFrameAndSetE1Offset( Frame parent, double offset) {

		// Create a new frame
		Frame childFrame = new Frame(parent);

		// Create new redundancy information
		LBRE1Redundancy newRedundancyInformation = new LBRE1Redundancy().setE1(offset);

		// Add new redundancy information to new frame
		childFrame.setRedundancyInformation(lbr, newRedundancyInformation);
		return childFrame;
	}

}
