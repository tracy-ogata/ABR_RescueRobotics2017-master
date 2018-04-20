/**
 * Rescue Robotics 2016 App
 * Developed by Cognitive Anteater Robotics Laboratory at University of California, Irvine
 * Controls wheeled robot through IOIO
 * Parts of code adapted from OpenCV blob follow
 * Before running, connect phone to IOIO with a bluetooth connection
 * If you would like to uncomment sections for message passing, first connect peer phones using wifi direct
 */
package abr.main;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import com.google.android.gms.common.ConnectionResult;
import com.google.android.gms.common.api.GoogleApiClient;
import com.google.android.gms.common.api.GoogleApiClient.ConnectionCallbacks;
import com.google.android.gms.common.api.GoogleApiClient.OnConnectionFailedListener;
import com.google.android.gms.location.LocationListener;
import com.google.android.gms.location.LocationRequest;
import com.google.android.gms.location.LocationServices;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.List;

import ioio.lib.api.DigitalOutput;
import ioio.lib.api.IOIO;
import ioio.lib.api.PulseInput;
import ioio.lib.api.exception.ConnectionLostException;
import ioio.lib.util.BaseIOIOLooper;
import ioio.lib.util.IOIOLooper;
import ioio.lib.util.IOIOLooperProvider;
import ioio.lib.util.android.IOIOAndroidApplicationHelper;

import static abr.main.Main_activity.Robots.CARLITO;
import static abr.main.Main_activity.Robots.DOC;
import static abr.main.Main_activity.Robots.MR;
import static abr.main.Main_activity.Robots.MRS;
import static java.lang.Math.PI;
import static java.lang.Math.cos;

public class Main_activity extends Activity implements IOIOLooperProvider, SensorEventListener, ConnectionCallbacks, OnConnectionFailedListener,
		CvCameraViewListener2 // implements IOIOLooperProvider: from IOIOActivity
{
	private final IOIOAndroidApplicationHelper helper_ = new IOIOAndroidApplicationHelper(this, this); // from IOIOActivity

	//for Minion:
	String fromMaster = "", toMaster = "";

	double[] lidarGPS = {0,0};
	Location destinationCoords;
	boolean autoModefromM = false;

	//for both:
	boolean initialFieldScan = true,
			isMannequinFound = false,
			scanModefromM = true;

	byte hex = (byte)0x90;

	boolean redRobot = true; //account for different gear ratios
	int forwardSpeed;
	int turningSpeed;
	int obstacleTurningSpeed;
	
	// ioio variables
	IOIO_thread_rover_4wd m_ioio_thread;
	
	//blob detection variables
	private CameraBridgeViewBase mOpenCvCameraView;
	private Mat mRgba;
	private Scalar mBlobColorRgba;
	private ColorBlobDetector mDetector;
	private Mat mSpectrum;
	private Scalar CONTOUR_COLOR;
	
	//app state variables
	private boolean autoMode;
	private boolean scanMode;
	
	//variables for logging
	private Sensor mGyroscope;
	private Sensor mGravityS;
	float[] mGravity;
	float[] mGyro;

	//location variables
	private GoogleApiClient mGoogleApiClient;
	private double curr_lat;
	private double curr_lon;
	private Location curr_loc;
	private LocationRequest mLocationRequest;
	private LocationListener mLocationListener;
	Location dest_loc;
	float distance = 0;
	
	//variables for compass
	private SensorManager mSensorManager;
	private Sensor mCompass, mAccelerometer;
	float[] mAcc;
	float[] mGeomagnetic;
	public float heading = 0;
	public float bearing;

	//ui variables
	TextView sonar1Text;
	TextView sonar2Text;
	TextView sonar3Text;
	TextView distanceText;
	TextView bearingText;
	TextView headingText;
	
	//sockets for message passing
	Boolean isClient = true;
	ServerSocket serverSocket;
	Socket socket;
	Socket clientSocket;
	DataInputStream dataInputStream;
	DataOutputStream dataOutputStream;
	
	//occupancy grid variables
	Location centerLocation;
	Location topLeft;
	Location bottomRight;
	Location bottomLeft;
	Location topRight;
	Location coords;

	//calculate corners function
    int dy;
    int dx;
    int r_earth;
    double latitude;
    double longitude;
    double new_latitude;
    double new_longitude;


	//lidar scan
	int Index = 1;
	
	//timers
	int pauseCounter = 0;
	int backCounter = 0;
	int backObstacleLeftCounter = 0;
	int backObstacleRightCounter = 0;
	Long startTime;
	Long currTime;
	
	//pan/tilt
	int panVal=1500;
	int tiltVal=1500;
	boolean panningRight = false;
	boolean tiltingUp = false;
	int panInc;
	int tiltInc;

	enum Robots {
		DOC, MR, MRS, CARLITO, CARLOS, CARLY, CARLA, CARLETON,
	}

	//choose robot//*************************************************************************change accordingly********************************************************
	Robots minion = DOC;
	
	// called to use OpenCV libraries contained within the app as opposed to a separate download
	static {
		if (!OpenCVLoader.initDebug()) {
			// Handle initialization error
		}
	}

    byte LIDAR_ADDRESS = 0x62;		//Default slave address
    int cal_cnt = 0;
    int dis = 0;

    //Registers
    private byte ACQ_COMMAND = 0x00;					//Receiver bias correction
    private byte[] STATUS = new byte[]{(byte) 0x01};	//System status
    private byte SIG_COUNT_VAL = (byte) 0x02;			//Measurement rate & range
    private byte ACQ_CONFIG_REG = (byte) 0x04;			//Measurement speed & accuracy
    private byte THRESHOLD_BYPASS = (byte) 0x1c;		//Detection sensitivity
    private byte FULL_DELAY_HIGH = (byte) 0x0f;			//Distance measurement high byte (cm)
    private byte FULL_DELAY_LOW = (byte) 0x10;			//Distance measurement low byte (cm)

    private byte[] measurementArray = new byte[] {FULL_DELAY_HIGH, FULL_DELAY_LOW};
    //private byte[] measurementArray = {FULL_DELAY_HIGH};

    private byte[] empy_read = new byte[] {};

    private DigitalOutput trigger_port;
    private PulseInput monitor_port;


    int trigger_pin = 38;
    int monitor_pin = 40;
    float pulseDistance;
	
	// called whenever the activity is created
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		
		requestWindowFeature(Window.FEATURE_NO_TITLE);
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		setContentView(R.layout.main);
		
		helper_.create(); // from IOIOActivity
		
		//set up opencv camera
		mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.color_blob_detection_activity_surface_view);
		mOpenCvCameraView.setCvCameraViewListener(this);
		mOpenCvCameraView.enableView();

		//initialize textviews
		sonar1Text = (TextView) findViewById(R.id.sonar1);
		sonar2Text = (TextView) findViewById(R.id.sonar2);
		sonar3Text = (TextView) findViewById(R.id.sonar3);
		distanceText = (TextView) findViewById(R.id.distanceText);
		bearingText = (TextView) findViewById(R.id.bearingText);
		headingText = (TextView) findViewById(R.id.headingText);
		
		//add functionality to autoMode button
		Button buttonAuto = (Button) findViewById(R.id.btnAuto);
		buttonAuto.setOnClickListener(new OnClickListener() {
			public void onClick(View v) {
				if (!autoMode) {
					v.setBackgroundResource(R.drawable.button_auto_on);
					autoMode = true;
					startTime = System.currentTimeMillis();
				} else {
					v.setBackgroundResource(R.drawable.button_auto_off);
					autoMode = false;
				}
			}
		});
		
		//set starting autoMode button color
		if (autoMode) {
			buttonAuto.setBackgroundResource(R.drawable.button_auto_on);
		} else {
			buttonAuto.setBackgroundResource(R.drawable.button_auto_off);
		}

		//find 4 corners**********************************************************************************
		topLeft = new Location("");
		topLeft.setLongitude(-117.826558);
		topLeft.setLatitude(33.643253);
		bottomRight = new Location("");
		bottomRight.setLongitude(-117.826044);
		bottomRight.setLatitude(33.643221);
		topRight = new Location("");
		topRight.setLongitude(-117.826558);
		topRight.setLatitude(33.643253);
		bottomLeft = new Location("");
		bottomLeft.setLongitude(-117.826044);
		bottomLeft.setLatitude(33.643221);

		
		//set up location listener
		mLocationListener = new LocationListener() {
			public void onLocationChanged(Location location) {
				curr_loc = location;
				distance = location.distanceTo(dest_loc);
				bearing = location.bearingTo(dest_loc);
			}
			@SuppressWarnings("unused")
			public void onStatusChanged(String provider, int status, Bundle extras) {
			}
			@SuppressWarnings("unused")
			public void onProviderEnabled(String provider) {
			}
			@SuppressWarnings("unused")
			public void onProviderDisabled(String provider) {
			}
		};
		
		//set up compass
		mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
	    mCompass= mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
	    mAccelerometer= mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
	    mGyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
	    mGravityS = mSensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);

		// phone must be Android 2.3 or higher and have Google Play store
		// must have Google Play Services: https://developers.google.com/android/guides/setup
		buildGoogleApiClient();
		mLocationRequest = new LocationRequest();
	    mLocationRequest.setInterval(2000);
	    mLocationRequest.setFastestInterval(500);
	    mLocationRequest.setPriority(LocationRequest.PRIORITY_HIGH_ACCURACY);
	    
	    //set speeds. adjust accordingly for your robot
	    if(redRobot){
	    	forwardSpeed = 120;//180;
	    	turningSpeed = 100;//90;//80;//70;//100;
	    	obstacleTurningSpeed = 100;//90;//75;//55;//50;//70;//100;
	    } else {
	    	forwardSpeed = 80;//110 ;
	    	turningSpeed = 50;//80;
	    	obstacleTurningSpeed = 30;//45;//60;//90;
	    }
	}
	//Method necessary for google play location services
	protected synchronized void buildGoogleApiClient() {
	    mGoogleApiClient = new GoogleApiClient.Builder(this)
	        .addConnectionCallbacks(this)
	        .addOnConnectionFailedListener(this)
	        .addApi(LocationServices.API)
	        .build();
	}
	//Method necessary for google play location services
	@Override
    public void onConnected(Bundle connectionHint) {
        // Connected to Google Play services
		curr_loc = LocationServices.FusedLocationApi.getLastLocation(mGoogleApiClient);
	    startLocationUpdates();
    }
	//Method necessary for google play location services
	protected void startLocationUpdates() {
	    LocationServices.FusedLocationApi.requestLocationUpdates(mGoogleApiClient, mLocationRequest, mLocationListener);
	}
	//Method necessary for google play location services
    @Override
    public void onConnectionSuspended(int cause) {
        // The connection has been interrupted.
        // Disable any UI components that depend on Google APIs
        // until onConnected() is called.
    }
    //Method necessary for google play location services
    @Override
    public void onConnectionFailed(ConnectionResult result) {
        // This callback is important for handling errors that
        // may occur while attempting to connect with Google.
        //
        // More about this in the 'Handle Connection Failures' section.
    }
    @Override
	public final void onAccuracyChanged(Sensor sensor, int accuracy) {
		// Do something here if sensor accuracy changes.
	}
    
    //Called whenever the value of a sensor changes
	@Override
	public final void onSensorChanged(SensorEvent event) {
		 if(m_ioio_thread != null){
			  setText("ir1: "+m_ioio_thread.get_ir1_reading(), sonar1Text);
			  setText("ir2: "+m_ioio_thread.get_ir2_reading(), sonar2Text);
			  setText("ir3: "+m_ioio_thread.get_ir3_reading(), sonar3Text);
			  setText("distance: "+distance, distanceText);
			  setText("bearing: "+bearing, bearingText);
			  setText("heading: "+heading, headingText);
		  }
		 
		  if (event.sensor.getType() == Sensor.TYPE_GRAVITY)
			  mGravity = event.values;
		  if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE)
			  mGyro = event.values;
		  if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER)
		      mAcc = event.values;
		  if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD)
		      mGeomagnetic = event.values;
		  if (mAcc != null && mGeomagnetic != null) {
			  float[] temp = new float[9];
			  float[] R = new float[9];
			  //Load rotation matrix into R
			  SensorManager.getRotationMatrix(temp, null, mAcc, mGeomagnetic);
			  //Remap to camera's point-of-view
			  SensorManager.remapCoordinateSystem(temp, SensorManager.AXIS_X, SensorManager.AXIS_Z, R);
			  //Return the orientation values
			  float[] values = new float[3];
			  SensorManager.getOrientation(R, values);
			  //Convert to degrees
			  for (int i=0; i < values.length; i++) {
				  Double degrees = (values[i] * 180) / PI;
				  values[i] = degrees.floatValue();
			  }
			  //Update the compass direction
			  heading = values[0]+12;
			  heading = (heading*5 + fixWraparound(values[0]+12))/6; //add 12 to make up for declination in Irvine, average out from previous 2 for smoothness
		   }
	  }

	
	//Called whenever activity resumes from pause
	@Override
	public void onResume() {
		super.onResume();
	    if (mOpenCvCameraView != null)
			mOpenCvCameraView.enableView();
	    mSensorManager.registerListener(this, mCompass, SensorManager.SENSOR_DELAY_NORMAL);
	    mSensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_NORMAL);
	    mSensorManager.registerListener(this, mGyroscope, SensorManager.SENSOR_DELAY_NORMAL);
	    mSensorManager.registerListener(this, mGravityS, SensorManager.SENSOR_DELAY_NORMAL);
	    if (mGoogleApiClient.isConnected()) {
	        startLocationUpdates();
	    }
	}
	
	//Called when activity pauses
	@Override
	public void onPause() {
		super.onPause();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
		mSensorManager.unregisterListener(this);
		stopLocationUpdates();
	}
	
	protected void stopLocationUpdates() {
	    LocationServices.FusedLocationApi.removeLocationUpdates(mGoogleApiClient, mLocationListener);
	}
	
	//Called when activity restarts. onCreate() will then be called
	@Override
	public void onRestart() {
		super.onRestart();
		Log.i("activity cycle","main activity restarting");
	}

	//Called when camera view starts. change bucket color here
	public void onCameraViewStarted(int width, int height) {
		mRgba = new Mat(height, width, CvType.CV_8UC4);
		mDetector = new ColorBlobDetector();
		mSpectrum = new Mat();
		mBlobColorRgba = new Scalar(255);
		CONTOUR_COLOR = new Scalar(255, 0, 0, 255);

		mDetector.setHsvColor(new Scalar(13,214,214));
	}
	//Called when camera view stops
	public void onCameraViewStopped() {
		mRgba.release();
	}
	//Called at every camera frame. Main controls of the robot movements are in this function
	public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
		Log.i("hahaha","area:"+(mDetector.getMaxArea()/(mDetector.getCenterX()*mDetector.getCenterY()*4)) );
		
		mRgba = inputFrame.rgba();
		mDetector.process(mRgba);
		
		List<MatOfPoint> contours = mDetector.getContours();
		// Log.e("rescue robotics", "Contours count: " + contours.size());
		Imgproc.drawContours(mRgba, contours, -1, CONTOUR_COLOR);

		Mat colorLabel = mRgba.submat(4, 68, 4, 68);
		colorLabel.setTo(mBlobColorRgba);

		Mat spectrumLabel = mRgba.submat(4, 4 + mSpectrum.rows(), 70,
				70 + mSpectrum.cols());
		mSpectrum.copyTo(spectrumLabel);

		// majority of control exists here
		receive_from_M("AUTOMODE"); //add server stuff
		if (autoMode && (System.currentTimeMillis()-startTime < 900000)) { // only move if autoMode is on and time under time limit
			receive_from_M("DEST");//add server stuff
			dest_loc = destinationCoords;
			autoMode = autoModefromM;
			scanMode = scanModefromM;

			if(System.currentTimeMillis()-startTime > 9000000)
				autoMode = false;
			//did i do this right? ****************************************************************************************************
			if((minion == DOC || minion == MR || minion == MRS)  && initialFieldScan == true) {
				//if lidar is on pan/tilt
				if(curr_loc == topLeft || curr_loc == topRight || curr_loc == bottomLeft) {
					m_ioio_thread.set_speed(1500);
					m_ioio_thread.set_steering(1500);
					tiltVal = 1500;
					panVal = 1600;
					if (Index == 0 && panVal <= 1600) {
						panVal--;
                        double[] locationCoords = {curr_loc.getLatitude(),curr_loc.getLongitude()};
                        double [] lgpsCoords = calculateMannequinnGpsCoordinates(locationCoords[0],locationCoords[1],pulseDistance, heading);
						send_to_M(DOC, locationCoords, false, lgpsCoords);
					} else if (Index == 0 && panVal == 1400) {
						panVal = 1500;
						initialFieldScan = false;
					}

					Index++;
					Index = Index % 15;
				}
			}

			else if ((minion == CARLITO) && initialFieldScan == true) {
					//or if moving whole robot
				m_ioio_thread.set_speed(1500);
				if(curr_loc == bottomRight)
				{
					m_ioio_thread.set_speed(1500);
					m_ioio_thread.set_steering(1600);
					curr_loc.bearingTo(topRight);
				}
			}


			//scan(mRgba);
			if(backCounter > 0){
				m_ioio_thread.set_steering(1500);
				m_ioio_thread.set_speed(1500-forwardSpeed/2);//m_ioio_thread.set_speed(1500-forwardSpeed);
				panVal = 1500;
				tiltVal = 1500;
				backCounter--;
				if (backCounter == 0)
					pauseCounter = 5;
			}
			else if(backObstacleLeftCounter > 0){
				panVal = 1500;
				tiltVal = 1500;
				if (backObstacleLeftCounter > 10) {
					m_ioio_thread.set_speed(1500-obstacleTurningSpeed);
					m_ioio_thread.set_steering(1500);
				} else {
					m_ioio_thread.set_speed(1500-obstacleTurningSpeed);
					m_ioio_thread.set_steering(1600);
				}
				backObstacleLeftCounter--;
				if (backObstacleLeftCounter == 0)
					pauseCounter = 5;
			}
			else if(backObstacleRightCounter > 0){
				panVal = 1500;
				tiltVal = 1500;
				if (backObstacleRightCounter > 10) {
					m_ioio_thread.set_speed(1500-obstacleTurningSpeed);
					m_ioio_thread.set_steering(1500);
				} else {
					m_ioio_thread.set_speed(1500-obstacleTurningSpeed);
					m_ioio_thread.set_steering(1400);
				}
				backObstacleRightCounter--;
				if (backObstacleRightCounter == 0)
					pauseCounter = 5;
			}
			else if(pauseCounter > 0){
				m_ioio_thread.set_speed(1500);
				m_ioio_thread.set_steering(1500);
				pauseCounter--;
				if (pauseCounter == 0) {
					if(System.currentTimeMillis()-startTime > 870000){ // go to center at 9:30, might have to change this
						dest_loc = centerLocation;
						m_ioio_thread.set_speed(1500+forwardSpeed);
						m_ioio_thread.set_steering(1500);
						//Log coordinates if the area of the orange bucket is greater than .01 of the screen
						if(m_ioio_thread != null && (m_ioio_thread.get_ir2_reading() < 17
								|| m_ioio_thread.get_ir1_reading() < 17 || m_ioio_thread.get_ir3_reading() < 17
								|| (mDetector.getMaxArea()/(mDetector.getCenterX()*mDetector.getCenterY()*4) > .12))) {
							Log.v("app.main", "obstacle reached");
                            double[] locationCoords = {curr_loc.getLatitude(),curr_loc.getLongitude()};
                            double[] notUsed = {0,0};
                            send_to_M(DOC, locationCoords, true, notUsed);

						}
					}
					else {
						Log.v("app.main", "pause == 0");
						m_ioio_thread.set_speed(1500+forwardSpeed);
						m_ioio_thread.set_steering(1500);
						dest_loc = destinationCoords;

					}
				}
			}
			//color blob detection
			else if(m_ioio_thread != null && (m_ioio_thread.get_ir2_reading() < 17 || m_ioio_thread.get_ir1_reading() < 17
					|| m_ioio_thread.get_ir3_reading() < 17 || (mDetector.getMaxArea()/(mDetector.getCenterX()*mDetector.getCenterY()*4) > .12))) { //might have to change this value
				//if(curr_loc.distanceTo(dest_loc) <= 25 && m_ioio_thread.get_ir2_reading() < 30 && (mDetector.getMaxArea()/(mDetector.getCenterX()*mDetector.getCenterY()*4) > .01)) //bucket reached
				if(curr_loc.distanceTo(dest_loc) <= 70) { //bucket reached
					Log.v("app.main", "bucket reached");
					//backCounter = 5;
                    double[] locationCoords = {curr_loc.getLatitude(),curr_loc.getLongitude()};
                    double[] notUsed = {0,0};
                    send_to_M(DOC, locationCoords, true, notUsed);
					if(m_ioio_thread.get_ir1_reading() < m_ioio_thread.get_ir3_reading()) { //bucket on left
						Log.v("app.main", "bucket on left");
						backObstacleLeftCounter = 18;
					} else { //bucket on right
						Log.v("app.main", "bucket on right");
						backObstacleRightCounter = 18;
					}
				}else{ //avoiding obstacle
					if(m_ioio_thread.get_ir1_reading() < m_ioio_thread.get_ir3_reading()){ //obstacle on left
						Log.v("app.main", "obstacle on left");
						backObstacleLeftCounter = 18;
						//m_ioio_thread.set_speed(1500-obstacleTurningSpeed);
						//m_ioio_thread.set_steering(1400);
					} else { //obstacle on right
						Log.v("app.main", "obstacle on right");
						backObstacleRightCounter = 18;
						//m_ioio_thread.set_speed(1500-obstacleTurningSpeed);
						//m_ioio_thread.set_steering(1600);
					}
						
				}
			}
			else if(curr_loc.distanceTo(dest_loc) > 70) { // follow compass, might have to increase this to 10
				float bearingMod = bearing%360;
				float headingMod = heading%360;
				
				m_ioio_thread.set_speed(1500+forwardSpeed);
				if (bearingMod >= headingMod) {
					if (bearingMod - headingMod <= 180)
						m_ioio_thread.set_steering(1500+turningSpeed);
					else
						m_ioio_thread.set_steering(1500-turningSpeed);
					}
				else {
					if (headingMod - bearingMod <= 180)
						m_ioio_thread.set_steering(1500-turningSpeed);
					else
						m_ioio_thread.set_steering(1500+turningSpeed);
				}
			} else { // follow orange bucket
				double momentX = mDetector.getMomentX();
				double momentY = mDetector.getMomentY();
				int centerThreshold = (int) (.333 * mDetector.getCenterX());
				if(mDetector.blobsDetected() == 0){
					m_ioio_thread.set_speed(1600);
					m_ioio_thread.set_steering(1500);
				} else {
					if(momentX > centerThreshold){
						m_ioio_thread.set_speed(1600);
						m_ioio_thread.set_steering(1600);
					}
					else if(momentX < -centerThreshold){
						m_ioio_thread.set_speed(1600);
						m_ioio_thread.set_steering(1400);
					}
					else {
						m_ioio_thread.set_speed(1600);
						m_ioio_thread.set_steering(1500);
					}
				}
			}
		} else {
			m_ioio_thread.set_speed(1500);
			m_ioio_thread.set_steering(1500);
		}

		return mRgba;
	}


	//revert any degree measurement back to the -179 to 180 degree scale
	public float fixWraparound(float deg){
		if(deg <= 180.0 && deg > -179.99)
			return deg;
		else if(deg > 180)
			return deg-360;
		else
			return deg+360;
		  
	}
	
	//determine whether 2 directions are roughly pointing in the same direction, correcting for angle wraparound
	public boolean sameDir(float dir1, float dir2){
		float dir = bearing%360;
		float headingMod = heading%360;
		//return (Math.abs((double) (headingMod - dir)) < 22.5 || Math.abs((double) (headingMod - dir)) > 337.5);
		return (Math.abs((double) (headingMod - dir)) < 2.5 || Math.abs((double) (headingMod - dir)) > 357.5);
	}
	
	//set the text of any text view in this application
	public void setText(final String str, final TextView tv) 
	{
		  runOnUiThread(new Runnable() {
			  @Override
			  public void run() {
				  tv.setText(str);
			  }
		  });
	}

	// receives string from master and assigns data to variables
	void receive_from_M (String data) {
		String string_auto_mode = data.substring(data.indexOf("AUTOMODE"), data.indexOf("SCANMODE")),
				string_scan_mode= data.substring(data.indexOf("SCANMODE"), data.indexOf("DEST")),
				string_coords = data.substring(data.indexOf("DEST"), data.length());

		autoModefromM = string_auto_mode.contains("true");
		scanModefromM = string_scan_mode.contains("true");
		destinationCoords = getCoords(string_coords);
	}

	// sends minion's data to master
	void send_to_M (Robots name, double[] location, Boolean foundMann, double[] lgps) {

		// Add robot name
		toMaster = toMaster + "NAME: " + name;

		//REPLACE ## later w/ curr_loc.getLatitude & curr_loc.getLongitude
		toMaster = toMaster + "GPS[LAT:" + location[0] + ", LON:" + location[1] + "], ";

		// Add mannequin status
		toMaster = toMaster + "MAN: " + foundMann;

		// Add GPS coordinates of victim seen by LIDAR
		toMaster = toMaster + "LGPS[LAT:" + lgps[0] + ", LON:" + lgps[1] + "], ";
	}

	//requires tuning *********************************************************************************************8
	public Location calculateCorners(Location topLeft){
		dy = 100; //distance away in km
		dx = 100;
		r_earth = 6378; //radius of earth in km
		latitude = topLeft.getLatitude();
		longitude = topLeft.getLongitude();
		new_latitude = latitude + (dy / r_earth) * (180 / PI);
		new_longitude = longitude + (dx / r_earth) * (180 / PI) / cos(latitude * PI / 180);

		Location resultLoc = new Location("");
		resultLoc.setLongitude(new_longitude);
		resultLoc.setLatitude(new_latitude);
		return resultLoc;
	}

    public Location getCoords (String str) {
        int find_comma, find_colon;

        //find lat
        find_colon = str.indexOf(':');
        find_comma = str.indexOf(',');
        String first_num = str.substring(find_colon+1, find_comma-1);

        //cut out lat
        str = str.substring(find_comma + 1, str.length());

        //find lon
        find_colon = str.indexOf(':');
        find_comma = str.indexOf(']');
        String sec_num = str.substring(find_colon+1, find_comma-1);


        coords = new Location("");

        double d = Double.parseDouble(first_num);
        double d2 = Double.parseDouble(sec_num);

        coords.setLongitude(d2);
        coords.setLatitude(d);

        return coords;
    }


	/****************************************************** functions from IOIOActivity *********************************************************************************/

	/**
	 * Create the {@link IOIO_thread}. Called by the
	 * {@link IOIOAndroidApplicationHelper}. <br>
	 * Function copied from original IOIOActivity.
	 *
	 * */
	@Override
	public IOIOLooper createIOIOLooper(String connectionType, Object extra) {
		if (m_ioio_thread == null
				&& connectionType
						.matches("ioio.lib.android.bluetooth.BluetoothIOIOConnection")) {
			m_ioio_thread = new IOIO_thread_rover_4wd();
			return m_ioio_thread;
		} else
			return null;

	}

    class Looper extends BaseIOIOLooper {
        /**
         * Called every time a connection with IOIO has been established.
         * Typically used to open pins.
         *
         * @throws ConnectionLostException
         *             When IOIO connection is lost.
         *
         * @see ioio.lib.util.IOIOLooper#setup()
         */

    @Override
    protected void setup() throws ConnectionLostException, InterruptedException {
        showVersions(ioio_, "IOIO connected!");
//            twi = ioio_.openTwiMaster(1, TwiMaster.Rate.RATE_100KHz, false);

        trigger_port = ioio_.openDigitalOutput(trigger_pin, false);
        monitor_port = ioio_.openPulseInput(monitor_pin, PulseInput.PulseMode.POSITIVE);

        //configure for default mode, balanced performance
//            configure(LIDAR_ADDRESS,0);
        //Thread.sleep(100);
        enableUi(true);
    }

    /**
     * Called repetitively while the IOIO is connected.
     *
     * @throws ConnectionLostException
     *             When IOIO connection is lost.
     * @throws InterruptedException
     * 				When the IOIO thread has been interrupted.
     *
     * @see ioio.lib.util.IOIOLooper#loop()
     */


    @Override
    public void loop() throws ConnectionLostException, InterruptedException {
        Log.i("LIDAR","---loop start---");

        // getDuration returns pulse width in seconds; 10us = 1cm
        // getDurations [s] * (1000000 us / 1s) * (1 cm / 10 us) * (1E-3 km / 1E2 cm) = 1 km
        pulseDistance = monitor_port.getDuration();
        if(pulseDistance >= 0.04)
        {
            pulseDistance = -1;  //didn't find anything.
        }

        Log.i("DISTANCE","" + pulseDistance + " km");

/*
            if (cal_cnt == 0) {
                dis = distance(LIDAR_ADDRESS,true);	//Measurement w/ bias correction
                //Log.i("MEASURED","bias");
            } else {
                dis = distance(LIDAR_ADDRESS,false); //Measurement w/o bias correction
                //Log.i("MEASURED","no bias");
            }
            //Increment reading counter
            cal_cnt++;
            cal_cnt = cal_cnt % 10;
            //Display distance reading
            Log.i("DISTANCE","" + dis + " cm");
            Thread.sleep(10);
*/
    }

    /**
     * Called when the IOIO is disconnected.
     *
     * @see ioio.lib.util.IOIOLooper#disconnected()
     */
    @Override
    public void disconnected() {
//            twi.close();
        monitor_port.close();
        enableUi(false);
        toast("IOIO disconnected");
    }

    /**
     * Called when the IOIO is connected, but has an incompatible firmware version.
     *
     * @see ioio.lib.util.IOIOLooper#incompatible(IOIO)
     */
    @Override
    public void incompatible() {
        showVersions(ioio_, "Incompatible firmware version!");
    }
}

    /**
     * A method to create our IOIO thread.
     *
     * @see ioio.lib.util.AbstractIOIOActivity#createIOIOThread()
     */
    @Override
    protected IOIOLooper createIOIOLooper() {
        return new Looper();
    }
    private void showVersions(IOIO ioio, String title) {
        toast(String.format("%s\n" +
                        "IOIOLib: %s\n" +
                        "Application firmware: %s\n" +
                        "Bootloader firmware: %s\n" +
                        "Hardware: %s",
                title,
                ioio.getImplVersion(IOIO.VersionType.IOIOLIB_VER),
                ioio.getImplVersion(IOIO.VersionType.APP_FIRMWARE_VER),
                ioio.getImplVersion(IOIO.VersionType.BOOTLOADER_VER),
                ioio.getImplVersion(IOIO.VersionType.HARDWARE_VER)));
    }
    private void toast(final String message) {
        final Context context = this;
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(context, message, Toast.LENGTH_LONG).show();
            }
        });
    }

    private int numConnected_ = 0;

    private void enableUi(final boolean enable) {
        // This is slightly trickier than expected to support a multi-IOIO use-case.
        runOnUiThread(new Runnable() {
            @Override
            public void run() {

            }
        });
    }

    // Takes latitude & longitude of current position (degree minutes, dm), sensor distance reading
    // (km), trueCourse (rad); Returns GPS coords as dm
    double[] calculateMannequinnGpsCoordinates(double lat_dm, double lon_dm, double distance_km, double trueCourse_rad) {
        double lat_rad = lat_dm * Math.PI / 180;
        double lon_rad = lon_dm * Math.PI / 180;
        double distance_rad = distance_km / 6371;       //6371 is the Earth's radius in km


        double lat = Math.asin(Math.sin(lat_rad)*Math.cos(distance_rad) + Math.cos(lat_rad)*Math.sin(distance_rad)*Math.cos(trueCourse_rad));
        double lon;

        if (Math.cos(lat) == 0) {
            lon = lon_rad;
        } else {
            lon = (((lon_rad - Math.asin(Math.sin(trueCourse_rad)*Math.sin(distance_rad) / Math.cos(lat))) + Math.PI) % (2*Math.PI)) - Math.PI;
        }

        lat = Math.toDegrees(lat);
        lon = Math.toDegrees(lon);

        double[] gps = {lat,lon};
        return gps;
    }


	@Override
	protected void onDestroy() {
		super.onDestroy();
		Log.i("activity cycle","main activity being destroyed");
		helper_.destroy();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
	}

	@Override
	protected void onStart() {
		super.onStart();
		Log.i("activity cycle","main activity starting");
		helper_.start();
		mGoogleApiClient.connect();
	}

	@Override
	protected void onStop() {
		Log.i("activity cycle","main activity stopping");
		super.onStop();
		helper_.stop();
		mGoogleApiClient.disconnect();
		try {
			if(socket != null)
				socket.close();
			if(serverSocket != null)
				serverSocket.close();
			if(clientSocket != null)
				clientSocket.close();
		} catch (IOException e) {
			Log.e("rescue robotics", e.getMessage());
		}
		
	}

	@Override
	protected void onNewIntent(Intent intent) {
		super.onNewIntent(intent);
			if ((intent.getFlags() & Intent.FLAG_ACTIVITY_NEW_TASK) != 0) {
			helper_.restart();
		}
	}
	
}
