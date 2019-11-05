/**
 * Contains all the processing done outside of the GRIP pileline
 */

package frc.robot.vision;

import java.util.ArrayList;
import java.util.Set;
import java.util.HashSet;
import java.util.Comparator;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.vision.*;
import edu.wpi.first.cameraserver.*;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;

import frc.robot.vision.GripPipeline;

/**
 * Manages all the vision tracking operations in a seperate thread
 */
public class Vision_Tracking implements Runnable{
    public enum Tracking_Param {
        HORIZONTAL,
        VERTICAL
    }

    // Calculation Paramters
    private static final int IMG_WIDTH = 1280;          // Width of the image
    private static final int IMG_HEIGHT = 720;          // Height of the image
    private final double ang_per_px = 60/IMG_WIDTH;     // Angle represented by each pixel

    private final double target_spacing = 8.75;                // Target Spacing in inches
    
    // Vision Object
    private int camera_index;                           // Index of the camera used for the vision tracking
    private UsbCamera camera;                           // Camera object used for vision tracking
    private CvSink camera_sink;                         // Camera Sink object used for vision tracking
    private Mat camera_image = new Mat();               // Image buffer for image captured for vision tracking
    private GripPipeline grip_pipeline;                 // Grip Pipeline
    private Thread vision_thread;                       // Thread used for vision tracking
    
    // Results
    private ArrayList<Target_Pair> target_pairs = new ArrayList<Target_Pair>();
    private final Object result_lock = new Object();    // Thread Lock object
    
    /**
     * Storage class for left and right hand vision target pairs
     */
    public class Target_Pair{
        private RotatedRect left;
        private RotatedRect right;

        /**
         * Initializes object with null values
         */
        public Target_Pair(){}

        /**
         * Initializes object with values
         * @param left      left target rectangle
         * @param right     right target rectangle
         */
        public Target_Pair(RotatedRect left, RotatedRect right){
            this.left = left;
            this.right = right;
        }

        /**
         * Retrieves the left target rectangle
         * @return  left target rectangle
         */
        public RotatedRect get_left(){
            return left;
        }

        
        /**
         * Retrieves the right target rectangle
         * @return  right target rectangle
         */
        public RotatedRect get_right(){
            return right;
        }
        
        /**
         * Sets the left target rectangle
         * @param left  left target rectangle
         */
        public void set_left(RotatedRect left){
            this.left = left;
        }

        /**
         * Sets the right target rectangle
         * @param right     right target rectangle
         */
        public void set_right(RotatedRect right){
            this.right = right;
        }

        /**
         * Retrieves the center point between the left and right target rectangles
         * @return  center point between the centers of the left and right target rectangle
         */
        public Point get_center(){
            Point center = new Point(0,0);
            
            center.x = (left.center.x + right.center.x) / 2;
            center.y = (left.center.y + right.center.y) / 2;

            return center;
        }

        /**
         * Retrieves the distances between the left and right target rectangles
         * @return  Distance between the centers of the left and right target rectangles
         */
        public double get_distance(){
            return get_distance(left.center, right.center);
        }

        /**
         * Checks if the supplied right rectangle is closer than the currently set right rectangle
         * @param right     new right rectangle to check
         * @return  true if the new right rectangle is closer than the existing right rectangle,
         *          or the existing right rectangle is null. False otherwise
         */
        public boolean is_closer(RotatedRect right){
            boolean closer = this.right == null;

            if(!closer){

                closer = get_distance() > get_distance(left.center, right.center);
            }

            return closer;
        }

        /**
         * Retrieves the Euclidean distance between two points
         * @param p1    first point
         * @param p2    second point
         * @return  the Euclidean distance between two points
         */
        public double get_distance(Point p1, Point p2){
            double x_diff = p1.x - p2.x;
            double y_diff = p1.y - p2.y;
            double dist = Math.sqrt(Math.pow(x_diff, 2) + Math.pow(y_diff, 2));

            return dist;
        }
    }

    /**
     * Comparator class for sorting Target_Pair objects by the distance between the left and right targets
     */
    public class sort_by_target_dist  implements Comparator<Target_Pair>{
        @Override
        public int compare(Target_Pair pair1, Target_Pair pair2){
            Point image_center = new Point(Vision_Tracking.IMG_WIDTH / 2, Vision_Tracking.IMG_HEIGHT / 2);

            double dist1 = pair1.get_distance(pair1.get_center(), image_center);
            double dist2 = pair2.get_distance(pair2.get_center(), image_center);

            return (int) (dist1 - dist2);
        }
    }

    /**
     * Comparator class for sorting Target_Pair objects by the distance from the center between the targets and the center of the image
     */
    public class sort_by_center_dist  implements Comparator<Target_Pair>{
        @Override
        public int compare(Target_Pair pair1, Target_Pair pair2){
            Point image_center = new Point(Vision_Tracking.IMG_WIDTH / 2, Vision_Tracking.IMG_HEIGHT / 2);

            double dist1 = pair1.get_distance(pair1.get_center(), image_center);
            double dist2 = pair2.get_distance(pair2.get_center(), image_center);

            return (int) (dist1 - dist2);
        }
    }


    /**
     * Vision Result container class
     */
    class Vision_Result{
        public boolean found = false;
        public double horizontal_angle = 0;
        public double vertical_angle = 0;
    }

    class Vision_PIDSource implements PIDSource{
        private final Vision_Tracking owner;
        private final Tracking_Param tracking_param;

        public Vision_PIDSource(Vision_Tracking owner, Tracking_Param tracking_param){
            this.owner = owner;
            this.tracking_param = tracking_param;
        }

        public Vision_Tracking get_owner(){
            return owner;
        }

        public Tracking_Param get_tracking_param(){
            return tracking_param;
        }

        @Override
        public PIDSourceType getPIDSourceType() {
            return PIDSourceType.kDisplacement;
        }

        @Override
        public void setPIDSourceType(PIDSourceType pidSource) {}

        @Override
        public double pidGet() {
            double result = 0;
            Vision_Result vision_result = owner.get_result();

            switch(tracking_param){
                case HORIZONTAL :
                    result = vision_result.horizontal_angle;
                    break;
                case VERTICAL :
                    result = vision_result.vertical_angle;
            }

            return result;
        }
    }

    /**
     * Constructor
     * @param camera_index  Index of the camera to use for the vision tracking
     * @param ang_per_px    Angle represented by each pixel of the camera
     */
    public Vision_Tracking(int camera_index, double ang_per_px) {
        // Init Camera
        this.camera_index = camera_index;
        camera = CameraServer.getInstance().startAutomaticCapture(camera_index);
        camera_sink = CameraServer.getInstance().getVideo();
        
        if(camera != null){
            camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
        }
        
        // Init Vision Pipeline
        grip_pipeline = new GripPipeline();

        // Init Vision Thread
        vision_thread = new Thread(this);
    }

    /**
     * Starts the Vision Processing thread
     */
    public void start(){
        vision_thread.start();
    }

    /**
     * Stops the Vision Processing thread
     * 
     * @param timeout timeout for waiting for the thread in milliseconds. Set to less than 0, timeout will be infinite.
     * @throws InterruptedException
     */
    public void stop(long timeout) throws InterruptedException {
        vision_thread.interrupt();

        if(timeout >= 0){
            vision_thread.join(timeout);
        }else{
            vision_thread.join();
        }
    }

    /**
     * Sorts the found vision target objects and matches target pairs
     */
    private void sort_targets(){
        ArrayList<RotatedRect> left_rectangles = new ArrayList<RotatedRect>();
        ArrayList<RotatedRect> right_rectangles = new ArrayList<RotatedRect>();
        
        ArrayList<MatOfPoint> contours = grip_pipeline.convexHullsOutput();

        // Check if no targets are found
        if(!contours.isEmpty()){
            // Sort all rectangles into left and right targets
            for(MatOfPoint contour : contours){
                MatOfPoint2f contour2f = new MatOfPoint2f(contour);
                RotatedRect rect = Imgproc.minAreaRect(contour2f);
                
                if(rect.angle < -45 && rect.angle > -100){
                    left_rectangles.add(rect);
                } 
                else if(rect.angle < 0 && rect.angle > -45){
                    right_rectangles.add(rect);
                }
            }

            // Match target pairs
            ArrayList<Target_Pair> full_pair_list = new ArrayList<Target_Pair>();
            for(RotatedRect left_rect : left_rectangles){
                Target_Pair pair = new Target_Pair();

                pair.set_left(left_rect);

                for(RotatedRect right_rect : right_rectangles){
                    if(pair.is_closer(right_rect)){
                        pair.set_right(right_rect);
                    }
                }
                
                full_pair_list.add(pair);
            }
            
            // Remove any duplicate RH matches
            synchronized (result_lock) {
                target_pairs.clear();
                full_pair_list.sort(new sort_by_target_dist());
                Set<RotatedRect> right_rect_list = new HashSet<RotatedRect>();
                for(Target_Pair pair: full_pair_list){
                    if(!right_rect_list.contains(pair.get_right())){
                        target_pairs.add(pair);
                        right_rect_list.add(pair.get_right());
                    }
                }

                // Find closest Pair to the center of the image
                target_pairs.sort(new sort_by_center_dist());
            }
        }
    }

    /**
     * Thread processing method
     */
    public void run(){
        while(!Thread.interrupted()){
            // Retrieve Camera 
            long result = camera_sink.grabFrameNoTimeout(camera_image);

            if(result == 0){
                // Run GRIP Pipeline
                grip_pipeline.process(camera_image);

                // Sort targets
                sort_targets();
            } else{
                synchronized(result_lock){
                    target_pairs.clear();
                }
            }
        }
    }

    /**
     * Checks if any targets have been found
     * @return
     */
    public Vision_Result get_result(){
        Vision_Result result = new Vision_Result();

        synchronized (result_lock) {
            result.found = !target_pairs.isEmpty();

            if(result.found){
                Point center = target_pairs.get(0).get_center();
                result.horizontal_angle = ang_per_px * (center.x - IMG_WIDTH / 2);
                result.vertical_angle = ang_per_px * (center.y - IMG_HEIGHT / 2);
            }
        }
        return result;
    }

    /**
     * Retrieves a PID source controlled by the vision tracking
     * @param tracking_param    vision tracking value to use for the PID source value
     * @return  new Vision_PIDSource object
     */
    public Vision_PIDSource get_pid_source(Tracking_Param tracking_param){
        return new Vision_PIDSource(this, tracking_param);
    }
}