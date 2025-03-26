package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;

import static frc.robot.Constants.DriverCameraConst;

/**
 * Sends a USB camera feed to the dashboard with additional graphics
 */
public class DriverCamera {
    private final Thread thread;
    private final UsbCamera camera;
    private final CvSink cvSink;
    private final CvSource outputStream;
    private Mat mat;

    /**
     * Contructor. Starts the camera thread.
     */
    public DriverCamera() {
        // Setup USB Camera
        camera = CameraServer.startAutomaticCapture();
        camera.setResolution(
            DriverCameraConst.width, 
            DriverCameraConst.height
        );     

        // Setup camera sink
        cvSink = CameraServer.getVideo();

        // Setup output stream to dashboard
        outputStream = CameraServer.putVideo(
            "Climber View", 
            DriverCameraConst.width, 
            DriverCameraConst.height
        );

        // Setup Stream thread
        thread = new Thread(this::execute);
        thread.setDaemon(true);
        
        // Start the camera thread
        start();
    }

    /**
     * Starts the camera thread
     */
    public void start() {
        thread.start();
    }

    /**
     * Stops the camera thread
     */
    public void stop() {
        thread.interrupt();
    }

    /**
     * Thread execution method
     */
    private void execute() {
        while(!Thread.interrupted()) {
            // Get new camera frame
            if(cvSink.grabFrame(mat) == 0){
                // Send the output the error.
                outputStream.notifyError(cvSink.getError());

                continue;
            }

            // Draw graphics
            Imgproc.line(
                mat, 
                DriverCameraConst.leftTop, 
                DriverCameraConst.leftBottom, 
                DriverCameraConst.lineColor,
                DriverCameraConst.lineWidth
            );
            
            Imgproc.line(
                mat, 
                DriverCameraConst.rightTop, 
                DriverCameraConst.rightBottom, 
                DriverCameraConst.lineColor,
                DriverCameraConst.lineWidth
            );

            // Put new frame to the driver station
            outputStream.putFrame(mat);
        }
    }
    
}
