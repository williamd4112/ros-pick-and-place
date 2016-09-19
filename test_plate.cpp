/********************************************************************
**                                                                 **
**     ChamferMatching                         - ver 4.3 -         **
**                                                                 **
**         Created by Ending2012 (Tsu-Ching Hsiao) on 2016/8/9     **
**                                                                 **
**        Copyright (c) 2012 End of APP. All rights reserved.      **
**                              E-mail: joe1397426985@gmail.com    **
*********************************************************************/

#define __CHAMFER_DEBUG_MODE___
#define __CHAMFER_INFO_REPORT___
#define __CHAMFER_LOW_MEMORY___
#define __MATCH_DEBUG_MODE___

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <ctime>
#include <cstdio>

//#include <Windows.h>
//#include <Psapi.h>

#include "Color.h"
#include "Contour.h"
#include "Detector.h"
#include "ChamferMatcher.h"
#include "INIParser.h"
#include "MatchingTest.h"


ending::Matcher::MatchPoints match(cv::Mat &image, cv::Mat &templ, ending::RChamferMatcher::MatcherConfig &mc){

	cv::Mat iedge;
	colorEdgeDetection(image, iedge, true);

	ending::RChamferMatcher cmatcher(mc);

	//ending::DEBUG_img = image.clone();  //if no defined __CHAMFER_DEBUG_MODE___  then remove this

	ending::Matcher::MatchPoints matchpoints;

	cmatcher.addMatcher(templ);
	cmatcher.matching(iedge,cv::Rect(0,0,image.size().width, image.size().height), matchpoints);

	//cv::imshow("debug", ending::DEBUG_img);   //if no defined __CHAMFER_DEBUG_MODE___  then remove this

	return matchpoints;
}

cv::Point getCVPoint(ending::INIparser &parser, const std::string &section, const std::string &name){
	std::string str = parser.getString(section, name);
	std::stringstream ss(str.substr(0, str.find(',')));
	int x, y;
	ss >> x;
	std::stringstream ss1(str.substr(str.find(',') + 1, std::string::npos));
	ss1 >> y;
	
	return cv::Point(x, y);
}

void getHolePos(ending::INIparser &parser, const std::string &section, std::map<char, cv::Point> &map){
	char c[1] = {};
	for (*c = 'A'; *c <= 'Z'; (*c)++){
		cv::Point p =  getCVPoint(parser, section, std::string(c));
		map[(*c)] = p;
	}
}

cv::Vec2d getCVVec2d(ending::INIparser &parser, const std::string &section, const std::string &name){
	std::string str = parser.getString(section, name);
	std::stringstream ss(str.substr(0, str.find(',')));
	double x, y;
	ss >> x;
	std::stringstream ss1(str.substr(str.find(',') + 1, std::string::npos));
	ss1 >> y;
	return cv::Vec2d(x, y);
}


void getRChamferMatcherConfig(ending::INIparser &parser, const std::string &section, ending::RChamferMatcher::MatcherConfig &mc){
	mc.setTemplScale(parser.getDouble(section, "SCALE"));
	mc.setMaxMatches(parser.getInt(section, "MAXMATCHES"));
	mc.setMinMatchDistance(parser.getDouble(section, "MINMATCHDISTANCE"));
	mc.setPadX(parser.getInt(section, "XSTEP"));
	mc.setPadY(parser.getInt(section, "YSTEP"));
	mc.setScales(parser.getInt(section, "SCALES"));
	mc.setMinScale(parser.getDouble(section, "MINSCALE"));
	mc.setMaxScale(parser.getDouble(section, "MAXSCALE"));
	mc.setOrientationWeight(parser.getDouble(section, "ORIENTATIONWEIGHT"));
	mc.setTruncate(parser.getDouble(section, "THRESHOLD"));
	mc.setAngularVelocity(parser.getDouble(section, "ROTATION"));
}

//change to plate scaled and rotated coordinate
cv::Point pointTrans(cv::Point puz, ending::Matcher::MatchPoint &plate){
	double sc = plate.getScale();
	puz.x = (int)((double)puz.x * sc + 0.5);
	puz.y = (int)((double)puz.y * sc + 0.5);
	ending::RotationMatrix rm(plate.getAngle());
	ending::Orient o;
	rm.rotate(puz, o);
	return puz;
}

//change to image coordinate and get bounding box
cv::Rect getHoleBoundingBox(cv::Point puz, double widthScale, double heightScale, ending::Matcher::MatchPoint &plate){
	puz = pointTrans(puz, plate);
	cv::Size size = plate.getBoundingBoxSize();
	size.width = (int)(size.width * widthScale + 0.5);
	size.height = (int)(size.height * heightScale + 0.5);
	puz.x -= size.width / 2;
	puz.y -= size.height / 2;
	return cv::Rect(puz.x, puz.y, size.width, size.height);
}

/*
int main(void){
	cv::Mat image = cv::imread("image.jpg");
	std::ifstream fin("config.ini");
	ending::INIparser parser(fin);

	if (parser.error()){
		std::cout << parser.errormsg() << std::endl;
		return 0;
	}

	cv::Mat templ = cv::imread(parser.getString("plate", "PATH"), CV_LOAD_IMAGE_GRAYSCALE);


	ending::RChamferMatcher::MatcherConfig mc;

	getRChamferMatcherConfig(parser, "plate", mc);
	ending::Matcher::MatchPoints matchpoints = match(image, templ, mc);





	if (matchpoints.size() <= 0){
		std::cout << "No any matching results in matcher" << std::endl;
		return 0;
	}

	ending::Matcher::MatchPoint &mp = matchpoints[0];
	cv::Point matchCenter = mp.getBoundingBoxCenter();
	double rotateAngle = mp.getAngle();
	double scaleSize = mp.getScale();
	double matchCost = mp.getCost();
	cv::Size matchSize = mp.getBoundingBoxSize();


	int maxRadius_ = (matchSize.width > matchSize.height ? matchSize.width : matchSize.height);
	maxRadius_ = (maxRadius_ + 1) / 2;
	cv::circle(image, matchCenter, maxRadius_, cv::Scalar(0, 255, 0));
	cv::Point lineEnd(matchCenter.x - (int)(sin(rotateAngle)*maxRadius_ + 0.5), matchCenter.y - (int)(cos(rotateAngle)*maxRadius_ + 0.5));
	cv::line(image, matchCenter, lineEnd, cv::Scalar(0, 255, 0));

	std::cout << "Best matching result: " << std::endl;
	std::cout << "\t Center: (" << matchCenter.x << ", " << matchCenter.y << ") " << std::endl;
	std::cout << "\t Box: (" << matchCenter.x - matchSize.width / 2 << ", " << matchCenter.y - matchSize.height / 2 << ") ";
	std::cout << "(" << matchCenter.x + matchSize.width / 2 << ", " << matchCenter.y + matchSize.height / 2 << ") " << std::endl;
	std::cout << "\t Angle: " << (rotateAngle / CV_PI * 180.0) << std::endl;
	std::cout << "\t Scale: " << scaleSize << std::endl;
	std::cout << "\t Cost: " << matchCost << std::endl;
	std::cout << std::endl;

	cv::imshow("image", image);

	cv::waitKey(0);
	return 0;
}*/

int main(void){

	std::ifstream fin("config.ini");
	ending::MatchingTest matchtest(fin);

	matchtest.init();
	
	cv::Mat image = cv::imread("image.jpg");

	cv::cvtColor(image, image, CV_BGR2HSV);
	matchtest.getImageH(image);

	cv::imshow("image", image);		

	cv::waitKey(0);
	return 0;
}

/*
int main(void){
	
	std::ifstream fin("config.ini");
	ending::MatchingTest matchtest(fin);

	matchtest.init();

	cv::Mat image = cv::imread("testimage.jpg");
	matchtest.setImage(image);

	std::vector<char> clist;
	clist.push_back('B');
	clist.push_back('H');
	clist.push_back('S');

	std::vector<cv::Rect> bbox;
	bbox.push_back(cv::Rect(360, 137, 68, 62));
	bbox.push_back(cv::Rect(390, 193, 82, 65));
	bbox.push_back(cv::Rect(423, 314, 66, 65));

	std::vector<ending::Matcher::MatchPoints> puzzmp = matchtest.matchPuzzle(clist, bbox);

	for (int i = 0; i < puzzmp.size(); i++){
		if (puzzmp[i].size() <= 0)continue;
		std::cout << "Puzzle " << i + 1 << std::endl;
		std::cout << "Angle: " << puzzmp[i][0].getAngle() << std::endl;
		std::cout << "Scale: " << puzzmp[i][0].getScale() << std::endl;
	}

	ending::Matcher::MatchPoints platemp = matchtest.matchPlate(cv::Rect(261, 104, 371, 352));

	if (platemp.size() > 0){
		std::cout << "Plate" << std::endl;
		std::cout << "Angle: " << platemp[0].getAngle() << std::endl;
		std::cout << "Scale: " << platemp[0].getScale() << std::endl;
	}
	

	ending::Matcher::MatchPoint holemp = matchtest.matchHole('R');

	
	std::cout << "Hole: " << std::endl;
	std::cout << "Angle: " << holemp.getAngle() << std::endl;
	std::cout << "Scale: " << holemp.getScale() << std::endl;

	


	cv::waitKey(0);

	return 0;
}*/




/*
std::vector<cv::Rect> list(){
	std::vector<cv::Rect> b;
	//b.push_back(cv::Rect(23, 160, 100, 93));
	//b.push_back(cv::Rect(117, 172, 93, 85));
	//b.push_back(cv::Rect(203, 171, 99, 90));
	//b.push_back(cv::Rect(287, 172, 84, 82));

	//b.push_back(cv::Rect(196,329,99,73));

	return b;
}

int main(void){
	cv::Mat image = cv::imread("image.jpg");
	//cv::Mat temp1 = cv::imread("F.png", CV_LOAD_IMAGE_GRAYSCALE);
	//cv::Mat temp2 = cv::imread("N1.png", CV_LOAD_IMAGE_GRAYSCALE);
	//cv::resize(temp1, temp1, cv::Size(temp1.size().width / 3, temp1.size().height / 3));

	ending::DEBUG_img = image.clone();

	cv::Mat iedge;
	//cv::Mat tedge1;
	//cv::Mat tedge2;
	colorEdgeDetection(image, iedge, true);
	cv::imwrite("edge.png", iedge);
	//edgeDetection(temp1, tedge1, false);
	//edgeDetection(temp2, tedge2, false);
	//cv::imshow("edge", iedge);
	//rotate(tedge, tedge, -10);
	//cv::imshow("temp edge1", tedge1);
	//cv::imshow("temp edge2", tedge2);
	//ending::debugimg = image.clone();

	ending::RChamferMatcher cmatcher(1, 20, 1.0, 3, 3, 3, 1.0, 1.2, 0.5, 20, 5);

	std::vector<cv::Rect> boundingBoxList = list();

	std::vector<ending::Matcher::MatchPoints> matchpoints;

	for (char a = 'N'; a <= 'N'; a++){
		char name[50];
		sprintf(name, "edge_x04/%c.png", a);
		cv::Mat templ = cv::imread(name, CV_LOAD_IMAGE_GRAYSCALE);
		std::cout << "Loading template: " << name << std::endl;
		cmatcher.addMatcher(templ);
	}

	//cv::chamerMatching(iedge, tedge, results, costs, 1, 20, 1.0, 3, 3, 5, 0.6, 1.6, 0.5, 20);


	double startTime, endTime;
	startTime = (double)clock();
	cmatcher.multimatching(iedge, boundingBoxList, matchpoints);

	endTime = (double)clock();

	cv::imwrite("Distance.png", ending::DEBUG_distimg);
	cv::imwrite("Orient.png", ending::DEBUG_orientimg);

	//PROCESS_MEMORY_COUNTERS beginpms, endpms;

	//GetProcessMemoryInfo(GetCurrentProcess(), &beginpms, sizeof(beginpms));

	for (int i = 0; i < matchpoints.size(); i++){
		if (matchpoints[i].size() <= 0){
			std::cout << "No any matching results in matcher [" << i << "] ..." << std::endl;
			continue;
		}

		ending::Matcher::MatchPoint &mp = matchpoints[i][0];
		cv::Point matchCenter = mp.getBoundingBoxCenter();
		double rotateAngle = mp.getAngle();
		double scaleSize = mp.getScale();
		double matchCost = mp.getCost();
		cv::Size matchSize = mp.getBoundingBoxSize();


		int maxRadius_ = (matchSize.width > matchSize.height ? matchSize.width : matchSize.height);
		maxRadius_ = (maxRadius_ + 1) / 2;
		cv::circle(image, matchCenter, maxRadius_, cv::Scalar(0, 255, 0));
		cv::Point lineEnd(matchCenter.x - (int)(sin(rotateAngle)*maxRadius_ + 0.5), matchCenter.y - (int)(cos(rotateAngle)*maxRadius_ + 0.5));
		cv::line(image, matchCenter, lineEnd, cv::Scalar(0, 255, 0));

		if (i == 0)std::cout << "1st ";
		else if (i == 1)std::cout << "2nd ";
		else if (i == 2)std::cout << "3rd ";
		else std::cout << i + 1 << "th ";

		std::cout << "Best matching result: " << std::endl;
		std::cout << "\t Center: (" << matchCenter.x << ", " << matchCenter.y << ") " << std::endl;
		std::cout << "\t Box: (" << matchCenter.x - matchSize.width / 2 << ", " << matchCenter.y - matchSize.height / 2 << ") ";
		std::cout << "(" << matchCenter.x + matchSize.width / 2 << ", " << matchCenter.y + matchSize.height / 2 << ") " << std::endl;
		std::cout << "\t Angle: " << (rotateAngle / CV_PI * 180.0) << std::endl;
		std::cout << "\t Scale: " << scaleSize << std::endl;
		std::cout << "\t Cost: " << matchCost << std::endl;
		std::cout << std::endl;
	}

	//GetProcessMemoryInfo(GetCurrentProcess(), &endpms, sizeof(endpms));

	//std::cout << "Peak Memory Usage: " << endpms.PeakWorkingSetSize - beginpms.WorkingSetSize << std::endl;
	//std::cout << "Memory Usage: " << endpms.WorkingSetSize - beginpms.WorkingSetSize << std::endl;
	std::cout << "Take: " << (endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;


    //cv::imwrite("result2.png", image);

	

	cv::imshow("result2", image);
	cv::imwrite("results.png", image);
	cv::imshow("asd", ending::DEBUG_img);
	cv::imwrite("debug.png", ending::DEBUG_img);
	cv::waitKey(0);
	return 0;
}
*/
