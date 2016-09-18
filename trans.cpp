
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <map>

#include "Color.h"
#include "Contour.h"
#include "ChamferMatcher.h"
#include "INIParser.h"

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
	ssl >> y;

	return cv::Vec2d(x,y);

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
cv::Rect getHoleBoundingBox(cv::Point puz, cv::Vec2d &scale, ending::Matcher::MatchPoint &plate){
	puz = pointTrans(puz, plate);
	cv::Size size = plate.getBoundingBoxSize();
	size.width = (int)(size.width * scale[0] + 0.5);
	size.height = (int)(size.height * scale[1] + 0.5);
	puz.x -= size.width/2;
	puz.y -= size.height/2;
	return cv::Rect(puz.x, puz.y, size.width, size.height);
}
