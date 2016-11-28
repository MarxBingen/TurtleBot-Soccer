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
#include "daniels/LogReg.h"

#define PI 3.14159265
#define MINANGLE -1.57079633
#define MAXANGLE 1.57079633


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

	dynamic_reconfigure::Server<daniels::ScanBallConfig> dc_srv;
	dynamic_reconfigure::Server<daniels::ScanBallConfig>::CallbackType dc_f;

	image_transport::Publisher imgpub;
	sensor_msgs::Image houghImg;
	bool publishHough;

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
	float minradius;		//min Radius des gesuchten Kreises in m
	float maxradius;		//max Radius des gesuchten Kreises in m
	float stepradius;       // erhöhungsschritt des radius bis maxradius
	int houghCircles;    	//Anzahl zu erzeugender Punkte im HoughRaum
	int minBallPeak;		//minimum peak um als Erkannt durchzugehen
	
	//LogReg
	std::vector<unsigned char> pointImageData;
	sensor_msgs::Image pointImage;
	bool useCostmapFilter;
	int logRegImageSize;		//28 cm default
	int logRegImageSizeHalf;	//14 cm default
	std::vector<LogisticRegression> logRegs;


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
	};
	houghPeak ball;
	//2-D-Hough-Welt
	std::vector<int> hv;

	//KONSTRUKTOR
	ScanBall(ros::NodeHandle n,std::string const& f) :
		nh(n),	//NodeHandle 
		//frame_id(f),
		laserSub(nh, "scan", 1),//LaserScan-Subscriber init
		laserNotifier(laserSub, listener, f, 1)	//MessageFilter, verarbeitet nur Messages, fuer die eine Transformation vorhanden ist
	{
		std::cout << "Initialisiere ScanBall-Parameter..." << std::endl;
		map_frame_id = "map";
		frame_id =f;
		scanResolution = 100;	//100cm / X = Auflösung in cm 
		processEveryScan = 15;	//jeder wievielte Scan verarbeitet werden soll
		processCounter = 1;		//Zaehler fuer processEveryScan
		minradius = 0.11f;      //min Radius des gesuchten Kreises in m
		maxradius = 0.12f;       //max Radius des gesuchten Kreises in m
		stepradius = 0.01f;       // erhoehungsschritt des radius bis maxradius
		houghCircles = 180;    	//Anzahl zu erzeugender Punkte im HoughRaum
		minBallPeak = 100;
		imgMaxX = 0;
		imgMinX = 0;
		imgMaxY = 0;
		imgMinY = 0;
		bild_counter = 0;    //damit nur alle x scans ein bild published wird
		publishHough = false;
		useCostmapFilter = true;
		//LogReg-Kram
		logRegImageSize = 28;	//28 cm default
		logRegImageSizeHalf = logRegImageSize/2;
		pointImage.height = logRegImageSize;
		pointImage.width = logRegImageSize;
		pointImage.step = pointImage.width; //wegen mono nur 1 byte
		pointImage.is_bigendian = false;
		pointImage.encoding = "mono8";
		pointImage.data.resize(pointImage.height*pointImage.step);
		pointImageData.resize(pointImage.height*pointImage.step);
		std::cout << "Erstelle Sin-Cos-Tabellen..." << std::endl;
		//Cos,Sin-Tabellen erstellen
		tableSinCircle = std::vector<float>(houghCircles);
		tableCosCircle = std::vector<float>(houghCircles);

		//Sinus und Cosinus-tabellen fuer Kreise befuellen
		for (int cs = 0; cs < houghCircles; cs++)
		{
			tableSinCircle[cs] = sin(360.0 / houghCircles * cs * PI / 180.0);
			tableCosCircle[cs] = cos(360.0 / houghCircles * cs * PI / 180.0);
		}

		std::cout << "Erstelle LogRegs..." << std::endl;
		initLogRegs();
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
		imgpub = it.advertise("hough_img_dev", 1000);
		//Subscriber fuer CostMap
		mapSub = nh.subscribe("move_base/global_costmap/costmap",1, &ScanBall::globalCostMapRecieved,this);
		std::cout << "Scanball gestartet !" << std::endl;
	}

	void initLogRegs()
	{
		unsigned char v[784] =  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 0, 255, 255, 255, 0, 255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		std::vector<unsigned char> vv(&v[0],&v[0]+784);
		std::vector<unsigned char> *pvv = &vv;
		logRegs.push_back(LogisticRegression());
		logRegs[0].learnFromExample(pvv,100,0.000005f);
		for (int i=0;i<784;i++)
			std::cout << logRegs[0].gewichte[i] << std::endl;
	}
	

	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
	{
		processCounter++;
		if (processCounter < processEveryScan)
			return;
		processCounter = 1;
		if (costmap == NULL && useCostmapFilter)
			return;
		sensor_msgs::PointCloud pcl;
		try
		{
			projector.transformLaserScanToPointCloud(map_frame_id, *scan_in, pcl, listener);
		}
		catch (tf::TransformException& e)
		{
			std::cout << e.what();
			//std::cout << "Transform to map nicht moeglich" << std::endl;
			return;
		}
		laserSub.unsubscribe();
		std::cout << ros::Time::now() << std::endl;
		std::fill(hv.begin(),hv.end(),0);
		ball.value=0;
		// Do something with cloud.
		imgMaxX = 0;
		imgMinX = hough_width; 
		imgMaxY = 0; 
		imgMinY = hough_height;
		float x,y;
		int dataCount = int(pcl.points.size());
		int data_x, data_y;
		int currentPoint = 0;
		int px=0,py=0;
		int imgStartIndex = 0;	//StartIndex des letzen Bilder
		int pixInImg = 0;
		int imgCounter = 0;
		int i=0,si=0;
		std::vector<int> fi;
		fi.resize(dataCount);
		int numFiltered=0;
		for (int pi=0;pi<dataCount;pi++)
		{	
			if (useCostmapFilter)
			{
				data_x = int((pcl.points[pi].x - costmap->info.origin.position.x) / costmap->info.resolution);
				data_y = int((pcl.points[pi].y - costmap->info.origin.position.y) / costmap->info.resolution);
				currentPoint = costmap->data[data_y * costmap->info.width + data_x];
			}
			if (!useCostmapFilter || currentPoint <=20 && currentPoint !=-1)
			{
				fi[numFiltered]=pi;
				numFiltered++;
			}
		}
		std::cout << "Points Costmap-Filtered:" << numFiltered << std::endl;
		while (si < numFiltered)
		{
			imgStartIndex = si;
			i=0;
			while (i < numFiltered)
			{
				px = int((pcl.points[fi[i]].x - pcl.points[fi[imgStartIndex]].x)*100.0f);
				py = int((pcl.points[fi[i]].y - pcl.points[fi[imgStartIndex]].y)*100.0f);
				if (abs(px) < logRegImageSizeHalf && abs(py) < logRegImageSizeHalf)
				{
					pointImage.data[((py + logRegImageSizeHalf) * logRegImageSize) + logRegImageSize - (logRegImageSizeHalf + px)]=255;
					pointImageData[((py + logRegImageSizeHalf) * logRegImageSize) + logRegImageSize - (logRegImageSizeHalf + px)]=255;
					pixInImg++;
				}
				i++;
			}
			if (pixInImg > 0)
			{
				imgpub.publish(pointImage);
				imgCounter++;
			}
			std::string i;
			std::cout << "Weiter ?" << std::endl;
			std::getline(std::cin,i);
			if (i.compare("abbrechen")==0)
				break;
			std::vector<unsigned char> *pid = &pointImageData;
			float p = logRegs[0].predict(pid);
			std::cout << "P:" << p << std::endl;
			std::fill(pointImageData.begin(),pointImageData.end(),0);
			pixInImg=0;
			std::fill(pointImage.data.begin(),pointImage.data.end(),0);
			si++;
		}
		std::cout << imgCounter << std::endl;
		std::cout << ros::Time::now() << std::endl;
		return;
		//BallPosition senden
		x = costmap->info.origin.position.x + ((ball.index_x / scanResolution));
		y = costmap->info.origin.position.y + ((ball.index_y / scanResolution));
		ballPoint.header.frame_id = map_frame_id;
		ballPoint.header.stamp = ros::Time().now();
		ballPoint.pose.position.x = x;
		ballPoint.pose.position.y = y;
		std::cout << "ballMax: " << ball.value << std::endl;
		std::cout << "ballPosition: " << x << ","<< y << std::endl;

		if (ball.value >= minBallPeak) {
			ballPub.publish(ballPoint);
		}
		if (publishHough){
			houghImage(ball.value,&houghImg);
			imgpub.publish(houghImg);
		}
	}
	
	void logReg(int startIndex){
		std::cout << "doing logReg..." << std::endl;
	}

	//transformiert den Punkt als Teil eines Kreises in den Houghraum
	void transformHoughCircle(float x, float y,float intensity)
	{
		//Es wird angenommen, dass die Koordinaten bereits in den Hough-Raum transformiert wurden
		int cx, cy;  //indizes im hough-raum
		int aIndex = 0;
		for (float radius = minradius; radius <= maxradius; radius += stepradius)
		{
			for (int p = 0; p<houghCircles; p++)
			{
				cx = x + (radius * scanResolution * tableCosCircle[p]);
				cy = y + (radius * scanResolution * tableSinCircle[p]);
				if (cx > imgMaxX) imgMaxX = cx;
				if (cx < imgMinX && cx > 0) imgMinX = cx;
				if (cy > imgMaxY) imgMaxY = cy;
				if (cy < imgMinY && cy > 0) imgMinY = cy;
				aIndex = (cy * hough_width) + cx;
				if (aIndex >= 0 && aIndex < hv.size())
				{
					//entfernungabhängige erhöhung ?? eigentlich besser, da sonst nahe Objekt bevorzugt werden
					//float rx,ry;
					//rx = x / scanResolution;
					//ry = y / scanResolution;
					//hv[cx][cy] += rx * rx + ry * ry;
					hv[aIndex] += (255 - intensity);
					if (hv[aIndex] > ball.value)
					{
						ball.value = hv[aIndex];
						ball.index_x = cx;
						ball.index_y = cy;
					}
				}
				else
					std::cout << "oob:" << cx << "," << cy << std::endl;
			}
		}
	}

	//erstellt aus dem hough-array ein bild
	void houghImage(int max_d, sensor_msgs::Image *img_out) 
	{
		std::cout << "Start Creating Hough Image" << std::endl;
		img_out->height = imgMaxY - imgMinY;
		img_out->width = imgMaxX - imgMinX;
		img_out->step = img_out->width; //wegen mono
		img_out->is_bigendian = false;
		img_out->encoding = "mono8";
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
				pxcolor = char((hv[aIndex] / max_value) * 255.0f);
				img_out->data[((py - imgMinY) * img_out->width) + (img_out->width - (px - imgMinX))] = pxcolor;
			}
		}
		std::cout << "created Hough Image" << std::endl;
	}

	void configCallback(daniels::ScanBallConfig &config, uint32_t level)
	{
		ROS_INFO("Rec request");
		scanResolution = config.scanResolution;
		minradius = config.minBallRadius / 1000.0f;
		maxradius = config.maxBallRadius / 1000.0f;
		stepradius = config.stepBallRadius / 1000.0f;

		houghCircles = config.houghCircleCount;

		minBallPeak = config.minBallPeak;

		publishHough = config.publishHough;

		processEveryScan = config.processEveryScan;
		processCounter = 1;
		//Cos,Sin-Tabellen neu erstellen
		tableSinCircle = std::vector<float>(houghCircles);
		tableCosCircle = std::vector<float>(houghCircles);
		//Sinus und Cosinus-tabellen für Kreise befüllen
		for (int n = 0; n < houghCircles; n++)
		{
			tableSinCircle[n] = sin(360.0 / houghCircles * n * PI / 180.0);
			tableCosCircle[n] = cos(360.0 / houghCircles * n * PI / 180.0);
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
