#include <iostream>
using namespace std;

#include <vector>

#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/ColorRGBA.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include "std_msgs/Float32.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"


#define angle_resolution 5//in degrees
#define distance_to_travel 1
#define angle_to_travel 20
#define precision 256
#define score_threshold 25

#define uncertainty 0.05

class Pos_Ori {
    private : 
        geometry_msgs::Point point;
        float angle;
        int score;
    
    public :
        Pos_Ori(float x, float y, float radian){
            point.x = x;
            point.y = y;
            angle = radian;
            score = 0.0;
        }

        Pos_Ori(float x, float y, float radian, int sc){
            point.x = x;
            point.y = y;
            angle = radian;
            score = sc;
        }
        
        Pos_Ori(){
            point.x = 0.0;
            point.y = 0.0;
            angle = 0.0;
            score = 0;
        }


        geometry_msgs::Point getPoint(){
            return point;
        }

        void setPoint(float x, float y){
            point.x = x;
            point.y = y;
        }

        float getAngle(){
            return angle;
        }

        void setAngle(float radian){
            angle = radian;
        }

        int getScore(){
            return score;
        }     
        
        void setScore(int s){
            score = s;
        }     
};

class project_node {

private:
    ros::NodeHandle n;

    ros::Subscriber sub_scan;
    ros::Subscriber sub_odometry;
    ros::Publisher pub_localization_marker;
    ros::Subscriber sub_position;

    // to store, process and display laserdata
    bool init_laser;
    int nb_beams;
    float range_min, range_max;
    float angle_min, angle_max, angle_inc;
    float r[1000], theta[1000];
    geometry_msgs::Point current_scan[1000];

    //to store the map
    nav_msgs::GetMap::Response resp;
    geometry_msgs::Point min, max;
    float cell_size;
    int width_max;
    int height_max;
    float ratio;

    // GRAPHICAL DISPLAY
    int nb_pts;
    geometry_msgs::Point display[1000];
    std_msgs::ColorRGBA colors[1000];

    bool init_odom;
    geometry_msgs::Point odom_current;
    float odom_current_orientation;
    geometry_msgs::Point odom_last;
    float odom_last_orientation;

    //to store the predicted and estimated position of the mobile robot
    bool localization_initialized;
    vector<Pos_Ori> valid_points;
    int valid_nodes_count;
    Pos_Ori final_predicted_position[16];
    Pos_Ori final_estimated_position[16];
    Pos_Ori robot_position; // changed every time robot moves, and at the initialisation
    
    // The weight of each iteration
    float weight[16];

    float distance_traveled;
    float previous_distance_traveled;
    float angle_traveled;
    float previous_angle_traveled;

public:

project_node() {

    sub_scan = n.subscribe("scan", 1, &project_node::scanCallback, this);
    sub_odometry = n.subscribe("odom", 1, &project_node::odomCallback, this);
    pub_localization_marker = n.advertise<visualization_msgs::Marker>("localization_marker", 1); // Preparing a topic to publish our results. This will be used by the visualization tool rviz
    // get map via RPC
    nav_msgs::GetMap::Request  req;
    ROS_INFO("Requesting the map...");
    while(!ros::service::call("static_map", req, resp)) {
      ROS_WARN("Request for map failed; trying again...");
      ros::Duration d(0.5);
      d.sleep();
    }

    init_odom = false;
    init_laser = false;
    localization_initialized = false;

    width_max = resp.map.info.width;
    height_max = resp.map.info.height;
    ratio = width_max / height_max;
    cell_size = resp.map.info.resolution;
    min.x = resp.map.info.origin.position.x;
    min.y = resp.map.info.origin.position.y;
    max.x = min.x + width_max*cell_size;
    max.y = min.y + height_max*cell_size;
    valid_nodes_count = 0;

    ROS_INFO("map loaded");
    ROS_INFO("Map: (%f, %f) -> (%f, %f) with size: %f",min.x, min.y, max.x, max.y, cell_size);
    ROS_INFO("wait for initial pose");

    //INFINTE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(10);// this node will work at 10hz
    while (ros::ok()) {
        ros::spinOnce();//each callback is called once
        update();
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }
}

//UPDATE: main processing of laser data
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update() {

    if ( init_odom && init_laser ) {
        previous_distance_traveled = distance_traveled;
        previous_angle_traveled = angle_traveled;

        distance_traveled = distancePoints(odom_current, odom_last);
        angle_traveled = odom_current_orientation-odom_last_orientation;
        if ( angle_traveled < -M_PI )
            angle_traveled += 2*M_PI;
        if ( angle_traveled > M_PI )
            angle_traveled -= 2*M_PI;
                
        if ( ( distance_traveled  != previous_distance_traveled ) || ( angle_traveled != previous_angle_traveled ) )
            ROS_INFO("distance_traveled = %f, angle_traveled = %f since last localization", distance_traveled, angle_traveled*180/M_PI);
        
        if ( !localization_initialized ) {
            estimate_initial_localization();
            localization_initialized = true;
            construct_weight();
            ROS_INFO("Init finished");
            return; 
    	}
    	
        if ( ( distance_traveled > distance_to_travel ) || ( fabs(angle_traveled*180/M_PI) > angle_to_travel ) )
        {
        	ROS_INFO("dt=%f, dtt=%d, at=%f, att=%d", distance_traveled, distance_to_travel, fabs(angle_traveled*180/M_PI), angle_to_travel);
        	update_position();
        	ROS_INFO("Press enter to continue");
        	getchar();
            bool result = estimate_updated_position();
            if(result) update_weight();
            else localization_initialized = false;
        }
    }

}// update

void estimate_initial_localization(){
    int length_count, height_count;
    float interval = 0.0;
    valid_points = first_filter(&length_count, &height_count, &interval);
    ROS_INFO("press enter to continue");
    getchar();
    int step = 1;
    while(valid_nodes_count >= 16){
        valid_points = second_filter(interval, step);
        step++;
        ROS_INFO("valid_nodes_count = %d, press enter to continue", valid_nodes_count);
    	getchar();
    }

    for(int i=0; i<valid_nodes_count; i++){
        final_estimated_position[i] = valid_points[i];
        if(final_estimated_position[i].getScore() > robot_position.getScore()){
            robot_position = final_estimated_position[i];
        }
    }
    
}

vector<Pos_Ori> first_filter(int *length_count, int *height_count, float *interval){
    float inter = 1.5;
    if(ratio < 1.0)
        inter = (max.x - min.x) / precision;
    else
        inter = (max.y - min.y) / precision;

    *length_count = width_max / inter;
    *height_count = height_max / inter;
    *interval = inter;
	
    ROS_INFO("First filter starts with interval %f, with width = %d and height = %d ", inter, width_max, height_max);
    ROS_INFO("press enter to continue");
    getchar();
    int score = 0;
    vector<Pos_Ori> valid_points;
    for(float currentX = min.x; currentX < max.x; currentX = inter + currentX){
    //ROS_INFO("currentX = %f", currentX);
        for (float currentY = min.y; currentY < max.y; currentY = inter + currentY){
            //ROS_INFO("currentY = %f", currentY);
            // Verify point valid or not
            if ( cell_value(currentX, currentY) == 0 ) {
                int highest_score = 0;
                float rad_for_highest_score = 0.0;
                for(float rad = -135 * M_PI / 180; rad <= 190 * M_PI / 180.0; rad += (45.0 * M_PI / 180)){
                    score = calculate_score(currentX, currentY, rad);
                    if(score >= score_threshold) {
                        if(score > highest_score){
                            highest_score = score;
                            rad_for_highest_score = rad;
                        }   
                    }
                }
                if(highest_score > 0){ // this handles the first time because otherwise the initial pts won't be updated
                    Pos_Ori *p = new Pos_Ori(currentX, currentY, rad_for_highest_score, highest_score);
                    valid_points.push_back(*p);
                }
            }
        }
    }
    valid_nodes_count = valid_points.size();
    ROS_INFO("Nb points left : %d\n", valid_nodes_count);
    
    return valid_points;
}

vector<Pos_Ori> second_filter(float interval, int step){
    ROS_INFO("Second filter starts with %d node with step = %f;\n", valid_nodes_count, (60 * M_PI) / (180 * pow(2, step)));	
    float x, y;
    float angle;
    vector<Pos_Ori> updated_points;
    
    for(int i=0; i<valid_points.size(); i++){
        x = valid_points[i].getPoint().x;
        y = valid_points[i].getPoint().y;
        angle = valid_points[i].getAngle();
        geometry_msgs::Point pts[5];
        float new_interval = interval / pow(3, step);
        //ROS_INFO("new_interval = %f", new_interval);

        pts[0].x = x;
        pts[0].y = y - new_interval;

        pts[1].x = x - new_interval;
        pts[1].y = y;

        pts[2].x = x;
        pts[2].y = y;

        pts[3].x = x + new_interval;
        pts[3].y = y ;

        pts[4].x = x;
        pts[4].y = y + new_interval;

        float min_orientation = angle - (180 * M_PI) / (180 * pow(2, step));
        float max_orientation = angle + (180 * M_PI) / (180 * pow(2, step));
        
        float steppy = (60.0 * M_PI) / (180 * pow(2, step));
        for(int m=0; m<5; m++){
            int score = 0, max_score = 0;
            float rad_of_max_score = 0.0;
            for(float rad = min_orientation; rad <= max_orientation; rad = rad + steppy){
				//ROS_INFO("Info : m = %d, pts[m] = (%f, %f), rad = %f, steppy=%f", m, pts[m].x, pts[m].y, rad, steppy);
				score = calculate_score(pts[m].x, pts[m].y, rad);
		        if(score > max_score){
		            max_score = score;
		            rad_of_max_score = rad;
                }
            }

            Pos_Ori *p = new Pos_Ori(pts[m].x, pts[m].y, rad_of_max_score, max_score);
            updated_points.push_back(*p);	
        }
    }
    
    // 0. create a new list of pts of interest 
    // 1. check the score for each pt, if it's below a certain score, dismiss (aka don't append to list of pts of interest) 
    // 2. return the list of pts of interest 
    vector<Pos_Ori> interest_points;
    vector<int> scores_interest;
    
    // 0. create a vector called 'scoreInts' 
    for (int i = 0; i<updated_points.size(); i++){
        // 1. Loop over the update pts and read for each pt its corresponding score
        scores_interest.push_back(updated_points[i].getScore());
        ROS_INFO("score[%d] : %d", i, updated_points[i].getScore());
    }

    sort(scores_interest.begin(), scores_interest.end(), greater<int>()); // sorts vector in descending order 
    ROS_INFO("size of scores_interest = %ld", scores_interest.size());

    int high_ninty_five = scores_interest[scores_interest.size() / 20]; // to get the 5th -> 100/20 = 5 -> this will be the threshold 
	ROS_INFO("Filtering point with scores less than %d\n", high_ninty_five);
    for (int i = 0; i<updated_points.size(); i++){
        if(updated_points[i].getScore() >= high_ninty_five){
            interest_points.push_back(updated_points[i]);
        }
    }
    valid_nodes_count = interest_points.size();
    ROS_INFO("Nb points left : %d\n", valid_nodes_count);
    return interest_points;


    // 2. store the corresponding score in the vector scoreints (append scoreInts)
    // 3. now that we have scoreInts vector of only integers which are the scores
    // 4. we can sort in descending order that list 
    // 5. and then we can get another loop to do the top 5 most percent:
        // 5.0 top-5 list 
        // 5.1 you loop over the updatepts 
        // 5.2 only get pts with corresponding scores -- if they match 
        // 5.3 append that list, return it 

}

void construct_weight(){ // when the 16 points are generated we calculate the weights of them (pts after the 1st and 2nd filters)
    int highScore = 0;
    for(int i=0; i<valid_nodes_count; i++){ // we get the highest score of all the points
        if(final_estimated_position[i].getScore() > highScore) 
            highScore = final_estimated_position[i].getScore(); 
    }
    for(int i=0; i<valid_nodes_count; i++){ // we calculate the weights 
        weight[i] = (final_estimated_position[i].getScore() + 0.0) / highScore;
        ROS_INFO("pt[%d] at (%f, %f) with ori = %f, score = %d, weight = %f", i, final_estimated_position[i].getPoint().x, final_estimated_position[i].getPoint().y, final_estimated_position[i].getAngle(), final_estimated_position[i].getScore(), weight[i]);
    }
}

void update_weight(){ // when the 16 points are generated we calculate the weights of them (pts after the 1st and 2nd filters)
    int highWScore = 0;
    for(int i=0; i<valid_nodes_count; i++){ // we get the highest weighted score of all the points
        if(final_estimated_position[i].getScore() * weight[i] > highWScore) 
            highWScore = final_estimated_position[i].getScore() * weight[i]; 
    }
    for(int i=0; i<valid_nodes_count; i++){ // we calculate the weights 
        weight[i] = final_estimated_position[i].getScore() * weight[i] / highWScore;
        if(weight[i] == 1.0) robot_position = final_estimated_position[i];
     	ROS_INFO("Update weight (%f, %f, %f) with score = %d, weight = %f", final_estimated_position[i].getPoint().x, final_estimated_position[i].getPoint().y, final_estimated_position[i].getAngle(), final_estimated_position[i].getScore(), weight[i]); 
    }
    ROS_INFO("Estimated robot postion at (%f, %f, %f) with score = %d", robot_position.getPoint().x, robot_position.getPoint().y, robot_position.getAngle(), robot_position.getScore()); 
}

void update_position() {
// NOTHING TO DO HERE


    odom_last = odom_current;
    odom_last_orientation = odom_current_orientation;
    
    //prediction of the current position of the mobile robot
    for(int i=0; i<valid_nodes_count; i++){
    	float ang = 0.0;
    	ang = final_estimated_position[i].getAngle() + angle_traveled;
		if ( ang < -M_PI )
		    ang += 2*M_PI;
		if ( ang > M_PI )
		    ang -= 2*M_PI;

    	final_predicted_position[i].setAngle(ang);
    	
    	float x = final_estimated_position[i].getPoint().x + distance_traveled*cos(ang);
    	float y = final_estimated_position[i].getPoint().y + distance_traveled*sin(ang);
    	
    	final_predicted_position[i].setPoint(x, y);
    	final_predicted_position[i].setScore(final_estimated_position[i].getScore());
		ROS_INFO("update position i from (%f, %f, %f) to (%f, %f, %f)", final_estimated_position[i].getPoint().x, final_estimated_position[i].getPoint().y, final_estimated_position[i].getAngle(), final_predicted_position[i].getPoint().x, final_predicted_position[i].getPoint().y, final_predicted_position[i].getAngle());
    }

    
}

bool estimate_updated_position(){ // estimate all the positions for the 16 pts
    ROS_INFO("update_position");

    //estimation of the positions closed to the predicted_position
    //we search the position with the highest sensor_model in a square of 1x1 meter around the predicted_position and with orientations around the predicted_orientation -M_PI/6 and +M_PI/6
    float min_x, max_x, min_y, max_y, min_orientation, max_orientation;
    for(int i=0; i<valid_nodes_count; i++){
 	//initialization of score_max with the predicted_position
        int ori_score_max = 0;
        if(cell_value(final_predicted_position[i].getPoint().x , final_predicted_position[i].getPoint().y) == 0)
			ori_score_max = calculate_score(final_predicted_position[i].getPoint().x , final_predicted_position[i].getPoint().y, final_predicted_position[i].getAngle());
        Pos_Ori *newP = new Pos_Ori(final_predicted_position[i].getPoint().x, final_predicted_position[i].getPoint().y, final_predicted_position[i].getAngle(), ori_score_max);

        min_x = final_predicted_position[i].getPoint().x - 0.5;
        max_x = final_predicted_position[i].getPoint().x + 0.5;
        min_y = final_predicted_position[i].getPoint().y - 0.5;
        max_y = final_predicted_position[i].getPoint().y + 0.5;
        min_orientation = final_predicted_position[i].getAngle() - 30.0 * M_PI/180.0;			// Lowest angle is -30 degree
        max_orientation = final_predicted_position[i].getAngle() + 30.0 * M_PI/180.0;			// Highest angle is 30 degree
		int score_max = ori_score_max;
        for(float x = min_x; x <= max_x; x+=0.05){
            for(float y = min_y; y <= max_y; y+=0.05){
            ROS_INFO("pos(%f, %f) in range (%f, %f) to (%f, %f)", x, y, min_x, min_y, max_x, max_y);
                if ( cell_value(x, y) == 0 ) { // robair can only be at a free cell
                    for(float rad = min_orientation; rad <= max_orientation; rad += ((5.0 * M_PI) / 180)){
						ROS_INFO("rad = %f, range = (%f, %f)", rad, min_orientation, max_orientation);
						int score_current = calculate_score(x, y, rad);
                        //we store the maximum score over all the possible positions in estimated_position
						
                        if( score_current > score_max ){
                            score_max = score_current;
                            newP->setPoint(x, y);
                            newP->setAngle(rad);
                            newP->setScore(score_current);
                        }
                        
                    }
                }
            }
        }
        final_estimated_position[i] = *newP;
        ROS_INFO("Iter %d completed, from (%f, %f, %f, %d) to (%f, %f, %f, %d)", i, final_predicted_position[i].getPoint().x, final_predicted_position[i].getPoint().y, final_predicted_position[i].getAngle(), ori_score_max, final_estimated_position[i].getPoint().x, final_estimated_position[i].getPoint().y, final_estimated_position[i].getAngle(), score_max);
        getchar();
    }  
    int max_score = 0;
    for(int i=0; i<valid_nodes_count; i++){
        if(final_estimated_position[i].getScore() > max_score)
            max_score = final_estimated_position[i].getScore();
    }
    if(max_score >= score_threshold) return true;
    else return false;
}

int calculate_score(float x, float y, float o)
{
//compute the score of the position (x, y, o)
	ROS_INFO("calculate_score(%f, %f, %f)", x, y, o*180/M_PI);
    nb_pts = 0;
    // we add the current hit to the hits to display
    display[nb_pts].x = x;
    display[nb_pts].y = y;
    display[nb_pts].z = 0;

    colors[nb_pts].r = 0;
    colors[nb_pts].g = 0;
    colors[nb_pts].b = 1;
    colors[nb_pts].a = 1.0;
    nb_pts++;

    // we add the current hit to the hits to display
    display[nb_pts].x = x+cos(o);
    display[nb_pts].y = y+sin(o);
    display[nb_pts].z = 0;

    colors[nb_pts].r = 1;
    colors[nb_pts].g = 1;
    colors[nb_pts].b = 0;
    colors[nb_pts].a = 1.0;
    nb_pts++;

    //loop over the hits of the laser
    int score_current = 0;
    float beam_angle = angle_min;
    for (int loop=0 ; loop < nb_beams; loop++, beam_angle += angle_inc)
    {
        //for each hit of the laser, we compute its position in the map and check if it is occupied or not
		
        geometry_msgs::Point hit;
        //hit.x = x + current_scan[loop].x * cos(o) + current_scan[loop].y * sin(o);
        //hit.y = y - current_scan[loop].x * sin(o) + current_scan[loop].y * cos(o);
        
        hit.x = x + current_scan[loop].x * cos(o) - current_scan[loop].y * sin(o);
        hit.y = y + current_scan[loop].x * sin(o) + current_scan[loop].y * cos(o);
		//ROS_INFO("sencing (%f, %f) of loop %d;", hit.x, hit.y, loop);
	    //getchar();
        // we add the current hit to the hits to display
        display[nb_pts] = hit;

        bool cell_occupied = false;
        //loop over the positions surronding the current hit of the laser and test if one of this cell is occupied
        for(float loop_x=hit.x-uncertainty; loop_x<=hit.x+uncertainty;loop_x += uncertainty)
            for(float loop_y=hit.y-uncertainty; loop_y<=hit.y+uncertainty;loop_y += uncertainty)
                //test if the current hit of the laser corresponds to an occupied cell
                cell_occupied = cell_occupied || ( cell_value(loop_x, loop_y) == 100 );

        if ( cell_occupied )
        {
            score_current++;

            // when matching is ok: the hit of the laser is green
            colors[nb_pts].r = 0;
            colors[nb_pts].g = 1;
            colors[nb_pts].b = 0;
            colors[nb_pts].a = 1.0;
        }
        //the current hit of the laser corresponds to a free cell
        else {
            // when matching is not ok: the hit of the laser is red
            colors[nb_pts].r = 1;
            colors[nb_pts].g = 0;
            colors[nb_pts].b = 0;
            colors[nb_pts].a = 1.0;
        }
        nb_pts++;
    }
    return(score_current);
}

int cell_value(float x, float y) {
//returns the value of the cell corresponding to the position (x, y) in the map
//returns 100 if cell(x, y) is occupied, 0 if cell(x, y) is free

    if ( ( min.x <= x ) && ( x <= max.x ) && ( min.y <= y ) && ( y <= max.y ) ) {
        float x_cell = (x-min.x)/cell_size;
        float y_cell = (y-min.y)/cell_size;
        int x_int = x_cell;
        int y_int = y_cell;
        //ROS_INFO("cell_value(%f, %f) at cell %d", x, y, width_max*y_int+x_int);
        return(resp.map.data[width_max*y_int+x_int]);
    }
    else
        return(-1);

}

//CALLBACK
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

    init_laser = true;
    // store the important data related to laserscanner
    range_min = scan->range_min;
    range_max = scan->range_max;
    angle_min = scan->angle_min;
    angle_max = scan->angle_max;
    angle_inc = scan->angle_increment;
    nb_beams = ((-1 * angle_min) + angle_max)/angle_inc;

    // store the range and the coordinates in cartesian framework of each hit
    float beam_angle = angle_min;
    for ( int loop=0 ; loop < nb_beams; loop++, beam_angle += angle_inc ) {
        if ( ( scan->ranges[loop] < range_max ) && ( scan->ranges[loop] > range_min ) )
            r[loop] = scan->ranges[loop];
        else
            r[loop] = range_max;
        theta[loop] = beam_angle;

        //transform the scan in cartesian framework
        current_scan[loop].x = r[loop] * cos(beam_angle);
        current_scan[loop].y = r[loop] * sin(beam_angle);
        current_scan[loop].z = 0.0;
    }

}//scanCallback

void odomCallback(const nav_msgs::Odometry::ConstPtr& o) {

    init_odom = true;
    odom_current.x = o->pose.pose.position.x;
    odom_current.y = o->pose.pose.position.y;
    odom_current_orientation = tf::getYaw(o->pose.pose.orientation);

}//odomCallback

//void positionCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& p) {

//    init_position = true;
 //   initial_position.x = p->pose.pose.position.x;
 //   initial_position.y = p->pose.pose.position.y;
//    initial_orientation = tf::getYaw(p->pose.pose.orientation);

//}

//GRAPHICAL_DISPLAY
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void populateMarkerTopic(){

    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "example";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;

    marker.color.a = 1.0;

    for (int loop = 0; loop < nb_pts; loop++) {
            geometry_msgs::Point p;
            std_msgs::ColorRGBA c;

            p.x = display[loop].x;
            p.y = display[loop].y;
            p.z = display[loop].z;

            c.r = colors[loop].r;
            c.g = colors[loop].g;
            c.b = colors[loop].b;
            c.a = colors[loop].a;

            //ROS_INFO("(%f, %f, %f) with rgba (%f, %f, %f, %f)", p.x, p.y, p.z, c.r, c.g, c.b, c.a);
            marker.points.push_back(p);
            marker.colors.push_back(c);
        }

    pub_localization_marker.publish(marker);

}

// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

}	

};

int main(int argc, char **argv){

    ros::init(argc, argv, "project_node");

    project_node bsObject;

    ros::spin();

    return 0;
}

