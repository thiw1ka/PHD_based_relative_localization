#include "lidarScan_robot_extractor/lidarScan_robot_extractor.hpp"

chrono::system_clock::time_point timer_start;
chrono::duration<double, std::milli> timer_duration;

void LidarObstacleExtraction::print_pair (vector<pair<double,double >>& points){

    cout<<"I am printing pairs---"<<endl;
    for(int T=0;T<points.size();T++){
        cout << points[T].first << " , "<< points[T].second << endl;
    }
	cout<<"printing pairs finished---"<<endl;
}

geometry_msgs::Point32 LidarObstacleExtraction::check_data_valid_for_robot (const vector<pair<double,double >>& data_set,const double& ang_incr) {
	double mean_dist = 0, delta_angle =0, arc_length = 0, theta = 0;
	int vec_size = data_set.size();
	geometry_msgs::Point32 pointEmpty;
		pointEmpty.x = 0.0;
		pointEmpty.y = 0.0;
		pointEmpty.z = 0.0;
	// auto sum = std::accumulate (data_set.begin(), data_set.end(), 0.0, [](pair <double, double> x){ return x.second;});
	/*calculating the mean out of all distances*/
	for(int k=0; k< vec_size; k++) {
		mean_dist += data_set[k].second; //calculating the sum of all distances
	}

	mean_dist = mean_dist / vec_size; // calculating mean
	if( obj_at_back == true ) {
		delta_angle = (ang_incr*vec_size);
		obj_at_back=false;
		//break;
		//cout<<"angle of object in the back ="<<delta_angle<<endl;
	}
	else {
		// angle difference 
		delta_angle = data_set[vec_size-1].first - data_set[0].first;
	} 

	arc_length = mean_dist * delta_angle; //* M_PI / 180.0;//what happen if angle is >180 just check
	//cout<<"length of detected robot = "<<arc_length<<endl;

	if ( arc_length < Max_RobWidth && arc_length > Min__RobWidth ) {
		std::printf("Its a robot \n");
		theta = data_set[ int( (vec_size - 1) / 2)].first; //getting the direction of the object
		double angleRad = theta;//*M_PI/180.0;
		geometry_msgs::Point32 p;
		p.x = mean_dist * std::cos(angleRad);
		p.y = mean_dist * std::sin(angleRad);
		p.z = 0.0;
		std::printf("checking lidar point_inside x: %f, y: %f, z: %f ",p.x, p.y, p.z);
		if (computeInsideBoxPtr->isPointInsideBox(homogMatrix,p)) {
			std::printf(": Inside \n");
			return p;
		}
		std::printf(": NOT_inside \n");

	}

	return pointEmpty;
}




void LidarObstacleExtraction::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){

	cout<<"----- Lidar new scan-----"<<1+counter<<endl;
	homogMatrix = odom_cb.get_homog();
	sensor_msgs::PointCloud msg;
	msg.header = scan->header;
	msg.points.clear();

	obj_at_back = false;
	int count = scan->scan_time / scan->time_increment;//cout<< "count at the begining = "<<count<<endl;
	if (scan->scan_time==0) {//in simulation the scan time is zero
		count=scan->ranges.size();
	}

	vector <pair<double,double >> Temp_LObjs;// temporary Lidar objects

	for(auto i = 0; i < count; i++) {

		float degree = 0; //= RAD2DEG(scan->angle_min + scan->angle_increment * i);

		//checking for object that lies in start/end border
		if (i == 0 && scan->ranges[count-1] > Min_ScanRange && scan->ranges[0] > Min_ScanRange) {
			double jumpAtStart = scan->ranges[i] - scan->ranges[count-1];
			bool foundi = false;
			if (jumpAtStart > -0.05 && jumpAtStart < 0.05) {
			// while ( jumpAtStart > -0.05 && jumpAtStart < 0.05 && foundi == false) {

				std::printf("checking jump start \n");
				int i_prev = (count-1);
				// find i in reverse direction via searching in the end of the data set
				for ( i = (count - 1); i > 0; i--) {
					if (scan->ranges[i] < Max_ScanRange && scan->ranges[i] > Min_ScanRange) {
						double jumpD = scan->ranges[i_prev]-scan->ranges[i]; 
						double jumpA = i_prev-i; // getting angle difference using via iteration

						if (jumpD < -0.05 || jumpD > 0.05 || jumpA >= 10) {
							//cout<<"found i true ="<<i_prev<<endl;
							foundi = true;
							i=i_prev;
							//cout<<"break"<<endl;
							break;
						}
						i_prev = i;
					}
				}
			}
			while (foundi == true) {
				std::printf("Foundi is true \n");
				int i_prev = i; /*resetting the start condition by changing the i*/
				Temp_LObjs.clear();
				for (i; i < count; i++) {
					degree = (scan->angle_min + scan->angle_increment * i);
					Temp_LObjs.push_back( make_pair(degree, scan->ranges[i]));			
				}
				count = i_prev;/*resetting the end condition*///cout<<" count ="<<count<<endl;
				i = 0;
				foundi = false;
				obj_at_back = true;
				break;
			}
		}
		if (scan->ranges[i] < Max_ScanRange && scan->ranges[i] > Min_ScanRange || i == (count-1)) {
			degree = (scan->angle_min + scan->angle_increment * i);
			Temp_LObjs.push_back( make_pair(degree,scan->ranges[i]) );
			int V_Size = Temp_LObjs.size();
			if (V_Size > 1) {
				double angle_diff = (Temp_LObjs[V_Size-2].first - Temp_LObjs[V_Size-1].first); //vector index starts from 0
				double dis_diff = (Temp_LObjs[V_Size-2].second - Temp_LObjs[V_Size-1].second);//getting distance of last two points
				if (dis_diff < -0.05 || dis_diff > 0.05 || i == (count-1)) {
				// if vector size less than 20 && jump is detected do following 
					Temp_LObjs.pop_back();
					//print(Temp_LObjs);
					if (Temp_LObjs.size() > Min_temp_vec_size) {
						auto point = check_data_valid_for_robot (Temp_LObjs, scan->angle_increment);
						if (point.x >5 || point.z > 2 || point.z < 0){
							std::printf("Angle %f degrees", RAD2DEG(degree));
							std::printf(", x: %f, y: %f, z: %f \n",point.x,point.y, point.z);
						}	
						double sum = point.x + point.y + point.z;
						if (sum != 0.0)	msg.points.push_back(point);
					}
					Temp_LObjs.clear();
					//cout<< "count at the end = "<<count<<" , i = "<<i<<endl;
					Temp_LObjs.push_back( make_pair(degree,scan->ranges[i]) );

					//call function to check the data set is valid
				}
			}
		}
		// std::printf("for loop: %i --- \n",i);
	}
	pub_locations.publish(msg); // publish lidar raw extractions
	blindspot ->checkPdForEachMeasurementFromLidar(msg, homogMatrix); //only pd allowed robot measurements will be remained
	pub_after_blindspot.publish(msg); //publish after blindspot check
	counter++;
	// std::printf("scan completed --- \n");

}