package com.example.manhattan;

import android.app.Activity;
import android.content.Context;
import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.View;
import android.widget.TextView;

import org.apache.commons.math3.filter.KalmanFilter;
import org.apache.commons.math3.filter.MeasurementModel;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import java.util.ArrayList;
import java.util.List;
import java.util.logging.Logger;

import static android.os.Build.VERSION_CODES.M;

import com.example.manhattan.R;

public class MainActivity extends Activity implements Orientation.Listener {
    private LocationManager locationManager;
    private LocationListener locationListener;
    private SensorManager sensorManager;

    // Sensors
    private Sensor accelerationSensor;
    private Sensor gravitySensor;
    private Sensor mangeticSensor;

    // Listeners
    private AccelerationSensorListener accelerationListener;
    private GravitySensorListener gravitySensorListener;
    private MagneticSensorListener magneticSensorListener;

    private TextView TextView_x;
    private TextView TextView_y;

    private TextView TextView_diff_x;
    private TextView TextView_diff_y;

    private Double correctedXVal;
    private Double correctedYVal;

    private List<Double> originalXVals;
    private List<Double> originalYVals;
    private List<Double> correctedXVals;
    private List<Double> correctedYVals;

    private TextView TextView_corrected_x;
    private TextView TextView_corrected_y;
    private TextView TextView_velocity;

    /* in radians*/
    volatile private double lat;
    volatile private double lng;

    /* in meters*/
    volatile private double x;
    volatile private double y;

    private double velocity;

    private double xV;
    private double yV;
    private double xA;
    private double yA;
    volatile private double xOrigin;
    volatile private double yOrigin;

    private double timestamp = 0;

    private float[] gravity;
    private float[] geomagnet;
    private double[] acceleration;

    private boolean text_visible = false;

    /* Kalman filter to correct GPS readings with acceleration */
    private KalmanFilter KF;

    /* interval of GPS reading in ms*/
    private final int interval = 1000;

    /* sampling interval of SENSOR_DELAY_GAME is 20ms */
    private final double sensorSamplingInterval = 0.1;

    private final float nanosecond = 1.0f / 1000000000.0f;

    private final Handler handler = new Handler();

    private Orientation mOrientation;
    private AttitudeIndicator mAttitudeIndicator;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        /* assume stationary when app opens*/
        xV = 0D;
        yV = 0D;
        xA = 0D;
        yA = 0D;

        this.TextView_x = (TextView) findViewById(R.id.x);
        this.TextView_y = (TextView) findViewById(R.id.y);

        this.TextView_corrected_x = (TextView) findViewById(R.id.corrected_x);
        this.TextView_corrected_y = (TextView) findViewById(R.id.corrected_y);

        this.TextView_diff_x = (TextView) findViewById(R.id.diff_x);
        this.TextView_diff_y = (TextView) findViewById(R.id.diff_y);

        this.TextView_velocity = (TextView) findViewById(R.id.Velocity);

        /* Initialize GPS Service*/
        locationManager = (LocationManager) getSystemService(Context.LOCATION_SERVICE);
        locationListener = new MyLocationListener();
        try {
            locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 200, 0, locationListener);
        } catch (SecurityException e) {

        }
        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);

        /* Initialize Accelerometer */
        accelerationSensor = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        accelerationListener = new AccelerationSensorListener();

        gravitySensor = sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);
        gravitySensorListener = new GravitySensorListener();

        mangeticSensor = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        magneticSensorListener = new MagneticSensorListener();

        sensorManager.registerListener(accelerationListener, accelerationSensor, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(gravitySensorListener, gravitySensor, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(magneticSensorListener, mangeticSensor, SensorManager.SENSOR_DELAY_GAME);

        originalXVals = new ArrayList<>();
        originalYVals = new ArrayList<>();
        correctedXVals = new ArrayList<>();
        correctedYVals = new ArrayList<>();

        mOrientation = new Orientation(this);
        mAttitudeIndicator = (AttitudeIndicator) findViewById(R.id.attitude_indicator);

    }

    @Override
    protected void onStart() {
        super.onStart();
        mOrientation.startListening(this);
    }

    @Override
    protected void onStop() {
        super.onStop();
        mOrientation.stopListening();
    }

    @Override
    public void onOrientationChanged(float pitch, float roll) {
        mAttitudeIndicator.setAttitude(pitch, roll);
    }

    public void onClick_StartTracking(View view) {

        /* Initialize Kalman Filter */
        KF = new KalmanFilter(new MyProcessModel(), new MyMeasurementModel());

        handler.postDelayed(new Runnable() {
            @Override
            public synchronized void run() {

                originalXVals.add(x);
                originalYVals.add(y);

                double[] measuredState = new double[] {x, y, xV, yV};
                KF.correct(measuredState);
                KF.predict();

                double[] stateEstimate = KF.getStateEstimation();
                correctedXVal = stateEstimate[0];
                correctedYVal = stateEstimate[1];
                correctedXVals.add(stateEstimate[0]);
                correctedYVals.add(stateEstimate[1]);

                redrawSeries();

                handler.postDelayed(this, interval);

            }
        }, 0);
    }

    public void redrawSeries() {

        this.TextView_x.setText("x " + x);
        this.TextView_y.setText("y " + y);
        this.TextView_corrected_x.setText("corrected x " + correctedXVal);
        this.TextView_corrected_y.setText("corrected y " + correctedYVal);
        this.TextView_diff_x.setText("difference x " + String.format("%.4f ", (x - correctedXVal)));
        this.TextView_diff_y.setText("difference y " + String.format("%.4f ", (y - correctedYVal)));
        this.TextView_velocity.setText("Velocity " + String.format("%.3f ", velocity));
        Log.i("apka", Double.toString(lat));
        Log.i("apka", Double.toString(lng));
    }

    public void onClick_SetOrigin(View view) {
        xOrigin = lng;
        yOrigin = lat;
        getXY(lat, lng);
        Log.i("apka", "on click changed " + lng);
    }

    /* uses radian */
    private synchronized void getXY(double lat, double lng) {
        double deltaLat = lat - yOrigin;
        double deltaLng = lng - xOrigin;

        double latCircumference = 40075160 * Math.cos(yOrigin);
        x = deltaLng * latCircumference / 2 / Math.PI;
        y = deltaLat * 40008000 / 2 / Math.PI;
    }

    public void onClick_ShowCorrected(View view) {
        if (text_visible) {
            this.TextView_corrected_x.setVisibility(View.INVISIBLE);
            this.TextView_corrected_y.setVisibility(View.INVISIBLE);
            this.TextView_diff_x.setVisibility(View.INVISIBLE);
            this.TextView_diff_y.setVisibility(View.INVISIBLE);
            text_visible = false;
        }
        else {
            this.TextView_corrected_x.setVisibility(View.VISIBLE);
            this.TextView_corrected_y.setVisibility(View.VISIBLE);
            this.TextView_diff_x.setVisibility(View.VISIBLE);
            this.TextView_diff_y.setVisibility(View.VISIBLE);
            text_visible = true;
        }

    }

    private class MyLocationListener implements LocationListener {
        @Override
        public void onStatusChanged(String s, int i, Bundle bundle) {

        }

        @Override
        public void onProviderEnabled(String s) {
        }

        @Override
        public void onProviderDisabled(String s) {

        }
        boolean is_first = true;
        @Override
        public void onLocationChanged(Location location) {
            lng = Math.toRadians(location.getLongitude());
            lat = Math.toRadians(location.getLatitude());
            Log.i("apka", "on location changed " + lng);
            getXY(lat, lng);
            if (is_first){
                xOrigin = lng;
                yOrigin = lat;
                getXY(lat, lng);
                is_first = false;
            }
        }
    }

    private class AccelerationSensorListener implements SensorEventListener {
        float[] rotationMatrix = new float[9];
        float[] inclineMatrix = new float[9];
        RealVector phoneAcceleration;
        RealMatrix R;
        RealMatrix I;

        double dT;

        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }

        @Override
        public synchronized void onSensorChanged(SensorEvent sensorEvent) {

            acceleration = castFloatToDouble(sensorEvent.values);
            acceleration[0] -= 0.000382033355;
            acceleration[1] += 0.000152450580;

            phoneAcceleration = MatrixUtils.createRealVector(acceleration);

            if(!(gravity == null || geomagnet == null)) {
                SensorManager.getRotationMatrix(rotationMatrix, inclineMatrix, gravity, geomagnet);

                R = MatrixUtils.createRealMatrix(resize3by3(castFloatToDouble(rotationMatrix)));
                I = MatrixUtils.createRealMatrix(resize3by3(castFloatToDouble(inclineMatrix)));

                phoneAcceleration = R.preMultiply(phoneAcceleration);
                phoneAcceleration = I.preMultiply(phoneAcceleration);


                double newXA = 0.8 * xA + 0.2 * phoneAcceleration.getEntry(0);
                double newYA = 0.8 * yA + 0.2 * phoneAcceleration.getEntry(1);


                if (timestamp != 0) {
                    dT = (sensorEvent.timestamp - timestamp) * nanosecond;



                    xV += (xA + newXA) * dT / 2;
                    yV += (yA + newYA) * dT / 2;
                    velocity = Math.sqrt(xV * xV + yV * yV);


                }
                Log.i("apka","phone acc x " + Double.toString(phoneAcceleration.getEntry(0)));
                Log.i("apka","phone acc y " + Double.toString(phoneAcceleration.getEntry(1)));
                Log.i("apka",Double.toString(xV));
                timestamp = sensorEvent.timestamp;

                xA = newXA;
                yA = newYA;

                if (Math.abs(xV) > 2.0) xV *= 0.6;
                if (Math.abs(yV) > 2.0) yV *= 0.6;


//                Log.v("data", xV + " " + yV + " " + dT);



            }



        }


        private double[][] resize3by3(double[] input) {
            if(input.length != 9) return null;

            double[][] result = new double[3][3];
            int index = 0;

            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    result[i][j] = input[index++];
                }
            }
            return result;
        }

        private double[] castFloatToDouble(float[] input) {
            double[] result = new double[input.length];
            int i = 0;
            for(float f : input) {
                result[i++] = (double) f;
            }

            return result;
        }
    }




    private class GravitySensorListener implements SensorEventListener {
        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }

        @Override
        public synchronized void onSensorChanged(SensorEvent sensorEvent) {
            gravity = sensorEvent.values;

        }
    }

    private class MagneticSensorListener implements SensorEventListener {
        @Override
        public synchronized void onSensorChanged(SensorEvent sensorEvent) {
            geomagnet = sensorEvent.values;
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }
    }

    private class MyProcessModel implements ProcessModel {


        @Override
        public RealMatrix getControlMatrix() {
            double[] control = {0, 0, 0, 0};
            return MatrixUtils.createRealDiagonalMatrix(control);
        }

        @Override
        public RealMatrix getInitialErrorCovariance() {
            double[] diagon = {0.01, 0.01, 1, 1};
            return MatrixUtils.createRealDiagonalMatrix(diagon);
        }

        @Override
        public RealMatrix getProcessNoise() {
            // assume no process noise first
            double[] processNoise = {0.001, 0.001, 5, 5};
            return MatrixUtils.createRealDiagonalMatrix(processNoise);
        }

        @Override
        public RealMatrix getStateTransitionMatrix() {
            /* assume constant velocity during interval*/
            double[][] stateTransitionMatrixData = {
                    {1, 0, interval / 1000.0, 0},
                    {0, 1, 0, interval / 1000.0},
                    {0, 0, 1, 0},
                    {0, 0, 0, 1}
            };

            return MatrixUtils.createRealMatrix(stateTransitionMatrixData);
        }

        @Override
        public synchronized RealVector getInitialStateEstimate() {
            double[] initialStateEstimate = {x, y, xV, yV};
            return MatrixUtils.createRealVector(initialStateEstimate);
        }
    }


    private class MyMeasurementModel implements MeasurementModel {
        @Override
        public RealMatrix getMeasurementMatrix() {
            return MatrixUtils.createRealIdentityMatrix(4);
        }

        @Override
        public RealMatrix getMeasurementNoise() {
            double[] measurementNoise = {0.2, 0.2, 0.5, 0.5};
            return MatrixUtils.createRealDiagonalMatrix(measurementNoise);
        }
    }
}