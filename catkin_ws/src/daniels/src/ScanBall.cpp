
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include "daniels/ScanBallConfig.h"

class ScanBall
{

public:

	ros::NodeHandle nh;
	laser_geometry::LaserProjection projector;

	tf::TransformListener listener;
	tf::StampedTransform transform;

	message_filters::Subscriber<sensor_msgs::LaserScan> laserSub;
	tf::MessageFilter<sensor_msgs::LaserScan> laserNotifier;

	ros::Publisher ballPub;
	geometry_msgs::PoseStamped ballPoint;
	geometry_msgs::PoseStamped lastBallPoint;
	geometry_msgs::PoseStamped currentBallPoint;

	dynamic_reconfigure::Server<daniels::ScanBallConfig> dc_srv;
	dynamic_reconfigure::Server<daniels::ScanBallConfig>::CallbackType dc_f;

	image_transport::Publisher imgpub;
	sensor_msgs::Image houghImg;
	bool publishHough;
	bool useCostmap;

	std::string frame_id;		//eigene Frame_ID
	std::string map_frame_id;	//Map_Frame_ID, normal = map
	
	ros::Subscriber mapSub;
	nav_msgs::OccupancyGrid::ConstPtr costmap;	//Die Costmap, mit der LaserPunkte gefiltert werden

	float scanResolution;	//2cm aufloesung, d.h. scan-werte kommen in Metern

	int hough_width;
	int hough_height;

	int processEveryScan;	//jeder wievielte Scan verarbeitet werden soll
	int processCounter;		//Zaehler fuer processEveryScan
	//Parameter Ballerkennnung
	float minRadius;		//min Radius des gesuchten Kreises in m
	float maxRadius;		//max Radius des gesuchten Kreises in m
	float stepRadius;       // erhöhungsschritt des radius bis maxradius
	int houghCircles;    	//Anzahl zu erzeugender Punkte im HoughRaum
	int minBallPeak;		//minimum peak um als Erkannt durchzugehen

	std::vector<float> tableSinCircle;
	std::vector<float> tableCosCircle;

	int imgMaxX;
	int imgMinX;
	int imgMaxY;
	int imgMinY;

	int bild_counter;    //damit nur alle x scans ein bild published wird

	//Stellt einen Peak im Houghraum dar
	struct houghPeak {
		int value;
		int index_x;
		int index_y;
		int pointIndex;
	};

	houghPeak ball;

	//2-D-Hough-Welt
	std::vector<int> hv;

	ScanBall(ros::NodeHandle n,std::string const& f) :
		nh(n),	//NodeHandle 
		laserSub(nh, "scan", 1),//LaserScan-Subscriber init
		laserNotifier(laserSub, listener, f, 1)	//MessageFilter, verarbeitet nur Messages, fuer die eine Transformation vorhanden ist
	{
		std::cout << "Initialisiere ScanBall-Parameter..." << std::endl;
		map_frame_id = "map";
		frame_id = std::string(f);
		scanResolution = 50.0f;	//100cm / X = Auflösung in cm 
		processEveryScan = 15;	//jeder wievielte Scan verarbeitet werden soll
		processCounter = 1;		//Zaehler fuer processEveryScan
		minRadius = 0.135f;      //min Radius des gesuchten Kreises in m
		maxRadius = 0.135f;       //max Radius des gesuchten Kreises in m
		stepRadius = 0.05f;       // erhoehungsschritt des radius bis maxradius
		houghCircles = 180;    	//Anzahl zu erzeugender Punkte im HoughRaum
		minBallPeak = 100;
		bild_counter = 0;    //damit nur alle x scans ein bild published wird
		houghImg.height = 10;
		houghImg.width = 10;
		houghImg.step = houghImg.width; //wegen mono
		houghImg.data.resize(houghImg.height*houghImg.step);
		publishHough = false;
		useCostmap = true;
		std::cout << "Erstelle Sin-Cos-Tabellen..." << std::endl;
		//Cos,Sin-Tabellen erstellen
		updateSinCosTables();
		//Subscriber fuer Laser
		std::cout << "Starte Publisher und Subscriber..." << std::endl;
		laserNotifier.registerCallback(boost::bind(&ScanBall::scanCallback, this, _1));
		laserNotifier.setTolerance(ros::Duration(0.1));
		//Publisher fuer Ballposition
		ballPub = nh.advertise<geometry_msgs::PoseStamped>("DetectedBall", 1);
		//Dynamic Reconfigure erstellen
		dc_f = boost::bind(&ScanBall::configCallback, this, _1, _2);
		dc_srv.setCallback(dc_f);
		//Hough_Image fuer Debug-Zwecke
		image_transport::ImageTransport it(nh);
		imgpub = it.advertise("hough_img_dev", 1);
		//Subscriber fuer CostMap
		mapSub = nh.subscribe("move_base/global_costmap/costmap",1, &ScanBall::globalCostMapRecieved,this);
		std::cout << "Scanball gestartet !" << std::endl;
	}

	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
	{
		processCounter++;
		if (processCounter < processEveryScan)
			return;
		processCounter = 1;
		if (useCostmap && costmap == NULL)
			return;
		if (minRadius > maxRadius)
		{
			std::cout << "Min Radius muss kleiner Max Radius sein !" << std::endl;
			return;
		}
		//in dieser PointCloud sind die map-transformierten Punkte
		sensor_msgs::PointCloud pcl;
		try
		{
			projector.transformLaserScanToPointCloud(map_frame_id, *scan_in, pcl, listener);
		}
		catch (tf::TransformException& e)
		{
			std::cout << e.what();
			return;
		}
		std::fill(hv.begin(),hv.end(),0);
		ball.value=0;
		// Do something with cloud.
		imgMaxX = 0;
		imgMinX = hough_width; 
		imgMaxY = 0; 
		imgMinY = hough_height;
		ros::Time startTime = ros::Time().now();
		float x,y;
		int dataCount = int(pcl.points.size());
		int i = 0;
		int data_x, data_y;
		int currentPoint = 0;
		int p_intensity = 1;
		float scanAngle = scan_in->angle_min;
		while (i < dataCount)
		{
			//nur bearbeiten, wenn nicht Obstacle in der GlobalCostMap
			if (useCostmap)
			{
				data_x = int((pcl.points[i].x - costmap->info.origin.position.x) / costmap->info.resolution);
				data_y = int((pcl.points[i].y - costmap->info.origin.position.y) / costmap->info.resolution);
				currentPoint = costmap->data[data_y * costmap->info.width + data_x];
			}
			if (!useCostmap || (currentPoint <= 20 && currentPoint !=-1))
			{
				//data_x = int((pcl.points[i].x - costmap->info.origin.position.x) * scanResolution);
				//data_y = int((pcl.points[i].y - costmap->info.origin.position.y) * scanResolution);
				data_x = scan_in->ranges[i]*scanResolution * cos(scanAngle);
				data_y = scan_in->ranges[i]*scanResolution * sin(scanAngle);

				for (int channel = 0;channel < pcl.channels.size();channel++)
				{
					if (pcl.channels[channel].name.compare("intensities")==0)
						p_intensity = pcl.channels[channel].values[i];
				}
				transformHoughCircle(data_x, data_y,p_intensity,scanAngle,i);
			}
			scanAngle+=scan_in->angle_increment;
			i++;
		}
		//BallPosition senden
		x = (float(ball.index_x)-(hough_width/2)) / scanResolution;
		y = (float(ball.index_y)-(hough_height/2)) / scanResolution;
		//x = pcl.points[ball.pointIndex].x;
		//y = pcl.points[ball.pointIndex].y;
		//ballPoint.header.frame_id = map_frame_id;
		ballPoint.header.frame_id = frame_id;
		ballPoint.header.stamp = scan_in->header.stamp;
		ballPoint.pose.position.x = x;
		ballPoint.pose.position.y = y;
		ballPoint.pose.orientation.w=1.0;
		try
		{
			listener.transformPose(map_frame_id,ballPoint,currentBallPoint);
		}
		catch (tf::TransformException ex)
		{
			std::cout << ex.what();
			return;
		}
		std::cout << "ProcTime:" << ros::Time().now() - startTime << std::endl;
		std::cout << "ballMax: " << ball.value << std::endl;
		std::cout << "ballPosition: " << x << ","<< y << std::endl;
		currentBallPoint.pose.orientation=getOrientation(lastBallPoint.pose.position,currentBallPoint.pose.position);
		lastBallPoint = currentBallPoint;
		if (ball.value >= minBallPeak) {
			ballPub.publish(currentBallPoint);
		}
		if (publishHough){
			houghImage(ball.value,&houghImg);
			imgpub.publish(houghImg);
		}
	}

	geometry_msgs::Quaternion getOrientation(geometry_msgs::Point& p1,geometry_msgs::Point& p2)
	{
		float a = atan2(p2.y-p1.y,p2.x-p1.x);
		geometry_msgs::Quaternion result;
		result.w = cos(a/2.0f);
		result.z = sin(a/2.0f);
		return result;
	}

	//transformiert den Punkt als Teil eines Kreises in den Houghraum
	void transformHoughCircle(float x, float y,int intensity,float angle_offset,int realIndex)
	{
		//Es wird angenommen, dass die Koordinaten bereits in den Hough-Raum transformiert wurden
		int cx, cy;  //indizes im hough-raum
		int aIndex = 0;
		int resAngleIndex = 0;
		angle_offset-=(M_PI/2.0f);
		int minC = (angle_offset)*(houghCircles/(2*M_PI));
		//std::cout << minC << std::endl;
		int maxHough = houghCircles / 2.0f; //nur 180 grad zeichnen
		for (float radius = minRadius; radius <= maxRadius; radius += stepRadius)
		{
			for (int p = 0 ;p<maxHough; p++)
			{
				resAngleIndex=minC+p;
				if (resAngleIndex>houghCircles)
					resAngleIndex-=houghCircles;
				if (resAngleIndex < 0)
					resAngleIndex+=houghCircles;
				cx = (hough_width/2.0f) + x + (radius * tableCosCircle[resAngleIndex]);
				cy = (hough_height/2.0f) + y + (radius * tableSinCircle[resAngleIndex]);
				if (cx > imgMaxX) imgMaxX = cx;
				if (cx < imgMinX && cx > 0) imgMinX = cx;
				if (cy > imgMaxY) imgMaxY = cy;
				if (cy < imgMinY && cy > 0) imgMinY = cy;
				aIndex = (cy * hough_width) + cx;
				if (aIndex >=0 && aIndex < hv.size())
				{
					hv[aIndex] += (255 - intensity);  //Entfernung
					if (hv[aIndex] > ball.value)
					{
						ball.value = hv[aIndex];
						ball.index_x = cx;
						ball.index_y = cy;
						ball.pointIndex = realIndex; 
					}
				}
				else
				{
					std::cout << "oob:" << resAngleIndex << "p:" << p << std::endl;
				}
			}
		}
	}

	//erstellt aus dem hough-array ein bild
	void houghImage(int max_d, sensor_msgs::Image *img_out) 
	{
		std::cout << "Start Creating Hough Image" << std::endl;
		std::cout << imgMinX << "-" << imgMaxX << "-" << imgMinY << "-" << imgMaxY << std::endl;
		int lastW = img_out->width;
		int lastH = img_out->height;
		if (imgMaxX-imgMinX < 10 || imgMaxY - imgMinY < 10)
			return;
		img_out->height = imgMaxY - imgMinY;
		img_out->width = imgMaxX - imgMinX;
		img_out->step = img_out->width; //wegen mono
		img_out->is_bigendian = false;
		img_out->encoding = "mono8";
		if (lastW != img_out->width || lastH != img_out->height)
			img_out->data.resize(img_out->height*img_out->step);
		int pxcolor = 0;
		int aIndex = 0;
		float max_value = float(max_d);
		for (int py = imgMinY; py<imgMaxY ; py += 1)
		{
			//X-Achse geht nach links ins plus in rviz
			for (int px = imgMinX; px<imgMaxX; px += 1)
			{
				aIndex = (py * hough_width) + px;
				if (aIndex >=0 && aIndex < hv.size())
				{
					pxcolor = char((hv[aIndex] / max_value) * 255.0f);
					img_out->data[((py - imgMinY) * img_out->width) + (px - imgMinX)] = pxcolor;
				}
			}
		}
		std::cout << "created Hough Image" << std::endl;
	}

	void configCallback(daniels::ScanBallConfig &config, uint32_t level)
	{
		ROS_INFO("Rec request");
		bool scanResChanged= (int(scanResolution)==config.scanResolution);
		scanResolution = float(config.scanResolution);
		minRadius = config.minBallRadius / 1000.0f;
		maxRadius = config.maxBallRadius / 1000.0f;
		stepRadius = config.stepBallRadius / 1000.0f;
		useCostmap = config.useCostmap;
		bool houghChanged = (houghCircles == config.houghCircleCount);
		houghCircles = config.houghCircleCount;
		if (houghChanged==0 || scanResChanged==0)
		{
			updateSinCosTables();
		}
		minBallPeak = config.minBallPeak;

		publishHough = config.publishHough;

		processEveryScan = config.processEveryScan;
		processCounter = 1;
		std::cout << "scanResolution: " << scanResolution << std::endl;
		std::cout << "minradius: " << minRadius << std::endl;
		std::cout << "maxradius: " << maxRadius << std::endl;
		std::cout << "stepradius: " << stepRadius << std::endl;
		std::cout << "minBallPeak: " << minBallPeak << std::endl;
		std::cout << "houghCircles: " << houghCircles << std::endl;
	}

	void updateSinCosTables()
	{
		std::cout << "Initialisiere SIN-COS-Tabellen neu..." << std::endl;
		//Cos,Sin-Tabellen neu erstellen
		tableSinCircle.resize(houghCircles);
		tableCosCircle.resize(houghCircles);
		//Sinus und Cosinus-tabellen für Kreise befüllen
		for (int n = 0; n < houghCircles; n++)
		{
			tableSinCircle[n] = sin(((2.0f*M_PI) / float(houghCircles) * float(n)));
			tableSinCircle[n] *= scanResolution;//optimierung
			tableCosCircle[n] = cos(((2.0f*M_PI) / float(houghCircles) * float(n)));
			tableCosCircle[n] *= scanResolution;//optimierung
			//std::cout << n << "=" << tableSinCircle[n] << "::" << tableCosCircle[n] << std::endl;
		}
	}

	void globalCostMapRecieved(const nav_msgs::OccupancyGrid::ConstPtr& map)
	{
		std::cout << "GlobalCostMap recieved" << std::endl;
		costmap = map;
		//resize hough-map abhaengig von costmap
		hough_width = (map->info.width * map->info.resolution) * scanResolution;
		hough_height = (map->info.height * map->info.resolution) * scanResolution;
		imgMinX=hough_width;
		imgMinY=hough_height;
		std::cout << "Hough-Raum: " << hough_width << " x " << hough_height << std::endl;
		hv.resize(hough_width * hough_height);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "my_scan_to_cloud");
	ros::NodeHandle n;
	std::string f(ros::this_node::getNamespace());
	f+= "/laser";
	f.erase(0,1);
	std::cout << "Starte ScanBall..." << std::endl;
	std::cout << "Laser Frame_id: " << f << std::endl;
	ScanBall lstopc(n,f);
	ros::spin();
	return 0;
}
