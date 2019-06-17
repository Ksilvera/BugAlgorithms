#include "BugAlgorithms.hpp"
#include <stdlib.h>

#define PI 3.14159265

int test = 0, hitGoalDist = -1;

double dist = -1;
int farenough = 0;
int fullLoop = 0;
int cx = 1, cy = 1;
double dist1 = -1;

BugAlgorithms::BugAlgorithms(Simulator * const simulator)
{
    m_simulator = simulator;   
    //add your initialization of other variables
    //that you might have declared
    stepsToClosest = 0;
    stepsFromClosest = 0;
    closestX, closestY = 0;
    leastDistance = -1;

    m_mode = STRAIGHT;
    m_hit[0] = m_hit[1] = HUGE_VAL;
    m_leave[0] = m_leave[1] = 0;
    m_distLeaveToGoal = HUGE_VAL;    
}

BugAlgorithms::~BugAlgorithms(void)
{
    //do not delete m_simulator  
}

Move BugAlgorithms::Bug0(Sensor sensor)
{
    //add your implementation
    //add your implementation
    Move move = {0,0};
    //Check if the robot has reached the goal
    if(m_simulator->HasRobotReachedGoal()){
        printf("\nSuccess!\n");
        return move;
    }
    //Find out robot location and goal location 
    double goalX = m_simulator->GetGoalCenterX();
    double goalY = m_simulator->GetGoalCenterY();
    double robotX = m_simulator->GetRobotCenterX();
    double robotY = m_simulator->GetRobotCenterY();
    double step = m_simulator->GetStep();
    double turn = m_simulator->GetWhenToTurn();

    //Grab tangent angle of goal and robot
    double goalTanAngle= fabs(atan2(goalY - robotY, goalX - robotX) * 180/PI);
    double distance = sqrt(pow(goalX-robotX,2) + pow(goalY-robotY,2));

    //Get Direction to the goal 
    double vectorX = (goalX-robotX)/ distance;
    double vectorY = (goalY-robotY)/ distance;
    double obsDistance = sqrt(pow(sensor.m_xmin-robotX,2) + pow(sensor.m_ymin-robotY,2));

    //Get the tangent if we are hitting the obstacle
    if(sensor.m_dmin <= turn){
        //Get the path to the goal
        double obsVectorX = (sensor.m_xmin-robotX) / obsDistance;
        double obsVectorY = (sensor.m_ymin-robotY) / obsDistance * -1;
        double obsTanAngle = fabs(atan2(sensor.m_ymin-robotY,sensor.m_xmin-robotX)*(180/PI));
        double diff = std::abs((int)(goalTanAngle - obsTanAngle) % 360);

        //Don't break away and go to the obstacle if angle too close to 90
        if(std::abs(diff) <= 90){
            vectorX = obsVectorY;
            vectorY = obsVectorX;
            //if we are going straight change to around
            if(m_mode == STRAIGHT){
                m_hit[0] = robotX;
                m_hit[1] = robotY;
                m_mode = AROUND;
            }
        }

        //if the difference of the angle is more than 90
        //then try to going to the obstacle 
        else{
            //take a step back and calculate the direction
            obsVectorX = robotX - sensor.m_xmin;
            obsVectorY = robotY - sensor.m_ymin;
            obsDistance = sqrt(pow(obsVectorY,2) + pow(obsVectorX,2));

            //normalize the direction
            obsVectorX = obsVectorX / obsDistance;
            obsVectorY = obsVectorY / obsDistance;

            //set the new move direction
            vectorX = obsVectorX;
            vectorY = obsVectorY;
        }
    }

    //check if the obstacle distance is greater than when to turn 
    if(obsDistance >= (turn/2)){

        //Change to the robot going straight 
        if(m_mode == AROUND){
            m_leave[0] = robotX;
            m_leave[1] = robotY;
            m_mode = STRAIGHT;
        }
        //set the move to the vector 
        move.m_dx = vectorX * step;
        move.m_dy = vectorY * step;
    }

    return move;
}

Move BugAlgorithms::Bug1(Sensor sensor)
{
    // Move only if the robot isn't close enough to the goal.

    Move move;
    double currX = m_simulator->GetRobotCenterX();
    double currY = m_simulator->GetRobotCenterY();

    if (m_simulator->HasRobotReachedGoal()) {
        // Stay in the same place since the goal was already reached.
        return move = {0,0};
    }

    double gX,gY,magnitude,stepSize;

    // Get step size from the simulator.
    stepSize = m_simulator->GetStep();

    switch (m_mode) {
        case STRAIGHT: {
            // Check for nearest obstacle.
            
            if(sensor.m_dmin > m_simulator->GetWhenToTurn() || fullLoop) {
            	fullLoop =0;
                // Calculate vector to goal.
                gX = m_simulator->GetGoalCenterX()
                - m_simulator->GetRobotCenterX();
                gY = m_simulator->GetGoalCenterY()
                - m_simulator->GetRobotCenterY();

                // Calculate the magnitude of the goal vector.
                magnitude = sqrt((gX*gX) + (gY*gY));

                // Normalize the vector to the goal and store as move.
                move = {(gX/magnitude) * stepSize,
                    (gY/magnitude) * stepSize};
                break;
            }
                farenough = 0;
 			fullLoop = 0;
 			cx = 1, cy = 1;
			dist1 = -1;
            // Hit an obstacle. Need to traverse around it.
            m_mode = AROUND;
            dist1 = m_simulator->GetDistanceFromRobotToGoal();
                    	//printf("INIT DIST %d\n", dist1);
            // Current location is a hit point.
            m_hit[0] = currX;
            m_hit[1] = currY;

            // The robot happens to be at the closest point seen so far.
            closestX = m_simulator->GetRobotCenterX();
            closestY = m_simulator->GetRobotCenterY();
            dist1 = m_simulator->GetDistanceFromRobotToGoal();

            // Calculate the collision vector.
            gX = sensor.m_xmin - closestX;
            gY = sensor.m_ymin - closestY;

            magnitude = sqrt((gX*gX) + (gY*gY));    
            // Robot stops 1 unit away from obstacles to avoid math.
            move = { gY/magnitude*stepSize, gX/magnitude*(-stepSize) };
        } break; // End of STRAIGHT segment.
        case STRAIGHT_AND_AWAY_FROM_LEAVE_POINT: {
            //
        } break; // End of S_A_W_F_L_P segment.
        case AROUND_AND_AWAY_FROM_HIT_POINT: {
            //
            double currX = m_simulator->GetRobotCenterX();
            double currY = m_simulator->GetRobotCenterY();
            // Calculate the collision vector.
            gX = sensor.m_xmin - currX;
            gY = sensor.m_ymin - currY;

            // Robot stops 1 unit away from obstacles to avoid math.
            move = { gY/magnitude*stepSize, gX/magnitude*(-stepSize) };
            if (m_simulator->IsPointNearLine(currX,currY,
                        m_leave[0],m_leave[1],
                        m_simulator->GetGoalCenterX(),
                        m_simulator->GetGoalCenterY())) {
                m_mode = STRAIGHT;
            }
        } break; // End of A_A_A_F_H_P segment.
        case AROUND_AND_TOWARD_LEAVE_POINT: {
            //
            double currX = m_simulator->GetRobotCenterX();
            double currY = m_simulator->GetRobotCenterY();
            // Calculate the collision vector.
            gX = sensor.m_xmin - currX;
            gY = sensor.m_ymin - currY;

            magnitude = sqrt((gX*gX) + (gY*gY));    
            // Robot stops 1 unit away from obstacles to avoid math.
            move = { gY/magnitude*stepSize, gX/magnitude*(-stepSize) };
            if (m_simulator->IsPointNearLine(currX,currY,
                        m_leave[0],m_leave[1],
                        m_simulator->GetGoalCenterX(),
                        m_simulator->GetGoalCenterY())) {
                m_mode = STRAIGHT;
            }
        } break; // End of A_A_T_L_P segment.
        case AROUND: {
        		stepsToClosest++; stepsFromClosest++;
       // 	printf("Steps to closest %d steps from close to hit %d\n", (stepsToClosest - stepsFromClosest), stepsFromClosest);

       // 	printf("000000000\n");
            //if ( (sensor.m_xmin == m_hit[0])
            //        && (sensor.m_ymin == m_hit[1])) {
if (farenough && m_simulator->ArePointsNear(currX,currY,m_hit[0],m_hit[1])){
        		fullLoop = 1;
        		//printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n\n");
        	}
        	if (!m_simulator->ArePointsNear(currX,currY,m_hit[0],m_hit[1]))
        		farenough = 1;

			//printf("%d\n", m_simulator->ArePointsNear(currX,currY,m_hit[0],m_hit[1]));

        	
        	//printf("current distance %f MIN DIST: %f\n", m_simulator->GetDistanceFromRobotToGoal(), dist1);
        	if(farenough){
            double dist = m_simulator->GetDistanceFromRobotToGoal();
            if (dist1 > m_simulator->GetDistanceFromRobotToGoal()){
                // This is the closest point known so far.
                //printf("%s\n", "x");
                double temp = m_simulator->GetDistanceFromRobotToGoal();
                dist1 = temp;
                closestX = currX;
                closestY = currY;
                stepsFromClosest = 0;
            }
        }



            if (fullLoop) {
                // Decide which way to go back towards the leave point.
                if ((stepsToClosest - stepsFromClosest) > stepsFromClosest) {
      				cx = -1;
      				cy = -1;
                }
                if(m_simulator->ArePointsNear(currX,currY,closestX,closestY)){
                	  gX = m_simulator->GetGoalCenterX()
                - m_simulator->GetRobotCenterX();
                gY = m_simulator->GetGoalCenterY()
                - m_simulator->GetRobotCenterY();

                // Calculate the magnitude of the goal vector.
                magnitude = sqrt((gX*gX) + (gY*gY));

                // Normalize the vector to the goal and store as move.
               	 move = {(gX/magnitude) * stepSize,
                    (gY/magnitude) * stepSize};
                    m_mode = STRAIGHT;
         		   break;
                }
            }

            // Move around the obstacle, along the tangent line.
            // Track the closest seen point.
            // Get distance from robot to goal.
            // Check if it is closer.

            // Haven't gone all the way around the obstacle yet.
            // Calculate the collision vector.
            gX = sensor.m_xmin - m_simulator->GetRobotCenterX();
            gY = sensor.m_ymin - m_simulator->GetRobotCenterY();

            magnitude = sqrt((gX*gX) + (gY*gY));    
            // Robot stops 1 unit away from obstacles to avoid math.
            move = { gY/magnitude*stepSize *cx, gX/magnitude*(-stepSize)*cy };
        } break; // End of AROUND segment.
    } // END OF SWITCH STATEMENT, NO DEFAULT STATEMENT.

    return move;
}

Move BugAlgorithms::Bug2(Sensor sensor)
{

    if (m_simulator->HasRobotReachedGoal())
    {
        //TERMINATE
        Move move = {0,0};
        printf("\nSUCCESS!\n");
        return move;
    }
    //init coordinates & step
    double R_initX = m_simulator->GetRobotInitX();
    double R_initY = m_simulator->GetRobotInitY();
    double step = m_simulator->GetStep();
    //goal coordinates
    double goalX = m_simulator->GetGoalCenterX();
    double goalY = m_simulator->GetGoalCenterY();
    //current robot's coordinates
    double R_curX = m_simulator->GetRobotCenterX();
    double R_curY = m_simulator->GetRobotCenterY();
    //check for if we're on M_Line and M_Line unit vector coordinates
    bool nearM_Line = m_simulator->IsPointNearLine(R_curX, R_curY, R_initX, R_initY, goalX, goalY);

    //change in x & y to make for step
    double dx = 0, dy = 0; 
/* Three cases:
 * 
 * 1) On the M_Line (not intersecting any obstacles) = Take a step along the M_Line
 * 2) Intersecting boundary:
 *      2.1) pick a direction, Left or Right (we'll always go left)
 *      2.2) follow obstacle (TRICKY) until back on M_Line
 *          2.2.1) IF youre close to init hitcoordinates(Using funct ArePointsNear(Hitpoint coordinates & current coordin)
 *             TERMINATE.u
            2.2.2) IF goal is reached WHILE following boundary TERMINATE
 * 3) On Goal = TERMINATE
*/

switch(m_mode)
{
    case STRAIGHT:
    {//Move along M_Line

        /* SEE EXPLAINATION FOR OBSTACLE\GOAL THETA BLOW
        *   Copied and pasted theta calculations to correct issue with spiral case.
        */
        //printf("DISTANCE FROM obstacle = %lf\n", m_simulator->GetDistanceFromRobotToGoal());
        double obstacleTheta = atan2(sensor.m_ymin - R_curY, sensor.m_xmin - R_curX) * 180 / (22.0/7.0);
        double goalTheta = atan2(goalY - R_curY, goalX - R_curX) * 180 / (22.0/7.0);
        int thetaDif = abs((int) (goalTheta - obstacleTheta) % 360);

        if(sensor.m_dmin > m_simulator->GetWhenToTurn() || (thetaDif > 80 && nearM_Line))
        { //if on M_Line, move along it
        //  printf("0000000\n");
            double M_LineX = (goalX - R_initX);
            double M_LineY = (goalY - R_initY);
            double euclid_dist = sqrt((M_LineX * M_LineX) + (M_LineY * M_LineY)); //magnitude
            M_LineX /= euclid_dist;
            M_LineY /= euclid_dist; 
            dx = M_LineX * step;
            dy = M_LineY * step;
            
        }
        else //moving along the M_Line led to an obstacle... we need to move around now
        { 
        //  printf("1111111\n");
            m_mode = AROUND;
            m_hit[0] = R_curX;
            m_hit[1] = R_curY;
            hitGoalDist = m_simulator->GetDistanceFromRobotToGoal();
            test = 0;
            double v_x = sensor.m_xmin - R_curX;
            double v_y = sensor.m_ymin - R_curY;
            double euclid_dist = sqrt((v_x * v_x) + (v_y * v_y));
            v_x /= euclid_dist;
            v_y /= euclid_dist;
            //converting the vector <v_x,v_y> to the left perpindicular unit vector V=<a,b> -> V=<-b,a>  
            dy = v_x * m_simulator->GetStep();
            dx = v_y * m_simulator->GetStep() * -1; 
            
        }
    }
    break;
    case AROUND:
        // Now we're in the business of going around.
        /* compares robot2obstacle theta to obot2goal theta 
        * if goalTheta - obstacleTheta > (sum number > 0) then we can move along m_line since the nearest obstacle isnt in the direction of the goal
        * if goalTheta - obstacleTheta is a small number than the goal and obstacles are in more or less the same direction
        * the goal is to have the robot between the goal and the obstacle, thus meaning we're free to move towards the obstacle
        */
        //arctangent(y,x) = angle from x axis (in radians). (Radian * 180 / (pi)) = degree
        double obstacleTheta = atan2(sensor.m_ymin - R_curY, sensor.m_xmin - R_curX) * 180 / (22.0/7.0);
        double goalTheta = atan2(goalY - R_curY, goalX - R_curX) * 180 / (22.0/7.0);
        int thetaDif = abs((int) (goalTheta - obstacleTheta) % 360);
        //in order to stop traveling around the obstacle the following should be true:
        // 1) we're near the m_line
        // 2) the difference between the obstacle and goal vector should be sufficent enough to
        //      indicate the possibility of an uninterrupted step should we move away from the obstacle
        // 3) only leave if we didnt just attempt to leave nearby
        // 4) only leave if the current distance from goal is closer to the goal than when we hit the obstacle (spiral case)
        if(nearM_Line && thetaDif > 85 && !(m_simulator->ArePointsNear(m_leave[0],m_leave[1], R_curX, R_curY)) && m_simulator->GetDistanceFromRobotToGoal() < hitGoalDist)
        {
            //printf("22222222\n");
            m_mode = STRAIGHT;
            m_leave[0] = R_curX;
            m_leave[1] = R_curY;
            double v_x = goalX - R_curX;
            double v_y = goalY - R_curY;
            double euclid_dist = sqrt((v_x * v_x) + (v_y * v_y));
            v_x /= euclid_dist;
            v_y /= euclid_dist;
            dx = v_x * m_simulator->GetStep();
            dy = v_y * m_simulator->GetStep();

        }
        else
        {
            if(!m_simulator->ArePointsNear(R_curX,R_curY,m_hit[0],m_hit[1]))
                {
                    test = 1;
                }
            if (test && m_simulator->ArePointsNear(R_curX,R_curY,m_hit[0],m_hit[1])){
                printf("Error: No path found.\nCurrent position is near init Hitpoint.\n");
                exit(1);
            }
        //  printf("333333333\n");
            double v_x = sensor.m_xmin - R_curX;
            double v_y = sensor.m_ymin - R_curY;
            double euclid_dist = sqrt((v_x * v_x) + (v_y * v_y));
            v_x /= euclid_dist;
            v_y /= euclid_dist;
            dy = v_x * m_simulator->GetStep();
            dx = v_y* m_simulator->GetStep() * -1;          

        }
        break;
    
}
    //printf("dx = %lf dy = %lf\n", dx,dy);
    Move move ={dx,dy}; //VALUE FOR WHERE TO MOVE
    return move;
}
