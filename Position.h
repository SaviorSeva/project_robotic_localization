#ifndef position
#define position

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
using namespace std;

class Position {
    private : 
        geometry_msgs::Point point;
        float angle;
        int score;
    
    public :
        position(float x, float y, float radian){
            point.x = x;
            point.y = y;
            angle = radian;
            score = 0.0;
        }

        position(float x, float y, float radian, int sc){
            point.x = x;
            point.y = y;
            angle = radian;
            score = sc;
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

        int getScore() {
            return score;
        }
}

#endif