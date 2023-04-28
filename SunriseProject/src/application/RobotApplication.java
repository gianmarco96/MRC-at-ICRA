package application;


import javax.inject.Inject;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.deviceModel.LBR;

import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.controllerModel.Controller;


/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class RobotApplication extends RoboticsAPIApplication {
	@Inject
	private LBR lBR_iiwa_14_R820_1;
	@Inject
	private MediaFlangeIOGroup mf;
	@Inject
	private Controller controller; 
	
	

	@Override
	public void initialize() {
		// initialize your application here
		mf = new MediaFlangeIOGroup(controller);
		mf.setOutput1(false);

		
	}

	@Override
	public void run() {
		// your application execution starts here
//		lBR_iiwa_14_R820_1.move(ptpHome());
//		mf.getInput1();
		getLogger().info("Starting");
	}
}