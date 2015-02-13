/*
 *  HandTracker.h
 *  
 *
 *  Created by Chau Nguyen on 6/13/11.
 *  Copyright 2011 Cornell University. All rights reserved.
 *
 */
#ifndef HANDTRACKER_H
#define HANDTRACKER_H

//standard c++ library
#include <math.h>
#include <utility>
#include <stdio.h>

// Headers for OpenNI
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

// Header for NITE
#include "XnVNite.h"

// local header
#include "PointDrawer.h"

//Header for graphcut
#include "graphcut.h"

//Header for image segmentation
#include "segment-image.h"

//image I/O
#ifdef DEBUG
#	include "pnmfile.h"
#endif

#if (USE_HAND_TRACKER == 1)
#	include <ApplicationServices/ApplicationServices.h>
#	define XML_PATH "../../Config.xml"
#	include <GLUT/glut.h>
#else
#	include "ConvexHull.h"
#	define XML_PATH "Config.xml"
#	if (XN_PLATFORM == XN_PLATFORM_MACOSX)
#		include <GLUT/glut.h>
#	else
#		include <GL/glut.h>
#	endif
#endif

#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
	{																\
		printf("%s failed: %s\n", what, xnGetStatusString(rc));		\
		return rc;													\
	}																\
	else printf("%s succeeded\n", what);							\

#define CHECK_ERRORS(rc, errors, what)		\
	if (rc == XN_STATUS_NO_NODE_PRESENT)	\
	{										\
		XnChar strError[1024];				\
		errors.ToString(strError, 1024);	\
		printf("%s\n", strError);			\
		return (rc);						\
	}

#include "signal_catch.h"


#define GL_WIN_SIZE_X 640
#define GL_WIN_SIZE_Y 480

using namespace std;
using namespace xn;

// OpenNI objects
Context g_Context;
DepthGenerator g_DepthGenerator;
HandsGenerator g_HandsGenerator;

// NITE objects
XnVSessionManager* g_pSessionManager;
SessionState g_SessionState = NOT_IN_SESSION;
XnVFlowRouter* g_pFlowRouter;

// the drawer
XnVPointDrawer* g_pDrawer;

// Draw the depth map
XnBool g_bDrawDepthMap = true;
XnBool g_bPrintFrameID = false;
const XnUInt32 historySize = 1;

// Use smoothing
XnFloat g_fSmoothing = 0.0f;
XnBool g_bPause = false;
XnBool g_bQuit = false;

//thresholds for bounding box size (proportional to cropDist)
const XnFloat cropSize = 80.0f;
const XnFloat cropDist = 800.0f;
const XnFloat cropOffset = cropSize * 0.25;

//graph cut constant
const unsigned int delta = 100;

//image segmentation constant
const float k = 50;
const float sigma = 0.01;
const float minSize = 150;

#ifdef DEBUG
unsigned int counter = 0;
char filename[200];
#endif

#if (USE_HAND_TRACKER == 1)
vector<XnPoint3D> buffer;
vector<int> states;
XnUInt16* depthbuffer = new XnUInt16[640*480];
float widthScale = 1;
float heightScale = 1;
int state = 0;
#endif

FILE * file = fopen("data/hole.txt", "a");

void getBoundingbox(const XnPoint3D& pt, XnUInt32& left, XnUInt32& right, 
					XnUInt32& top, XnUInt32& bottom){
	XnUInt32 size = cropSize * cropDist / pt.Z;
    XnUInt32 offset = cropOffset * cropDist / pt.Z;
    left = (pt.X - size > 0 ? pt.X - size : 0);
    right = (pt.X + size > GL_WIN_SIZE_X ? GL_WIN_SIZE_X : pt.X + size);
    top = (pt.Y - size > 0 ? pt.Y - size : 0);
    bottom = (pt.Y + size + offset > GL_WIN_SIZE_Y ? 
			  GL_WIN_SIZE_Y : pt.Y + size + offset); 
}

image<rgb>* cropHand(XnPoint3D& pt, DepthMetaData& depthMD, XnUInt32& left, XnUInt32& right, 
                     XnUInt32& top, XnUInt32& bottom, unsigned int& width, unsigned int& height)
{	
	width = right - left + 1;
	height = bottom - top + 1;
	
	image<rgb>* img = new image<rgb>(width, height);
	for (unsigned int y = 0; y < height; ++y)
		for (unsigned int x= 0; x < width; ++x){
			rgb color;
			int depth = depthMD(x + left, y + top);
			if (fabs(depth - pt.Z) > delta || depth > MAX_DEPTH || depth < MIN_DEPTH)
				color.r = 0;
			else color.r = UCHAR_MAX;
			color.g = color.r;
			color.b = color.r;
			
			img->data[y * width + x] = color;
		}
	
	return img;
}

void findSetSize(universe* u, unsigned int width, unsigned int height, map<int, int>& set_size){
	int corner1 = u->find(0);
	int corner2 = u->find(width-1);
	int corner3 = u->find((height-1)*width);
	int corner4 = u->find(height*width - 1);
	u->join(corner1, corner2);
	u->join(corner1, corner3);
	u->join(corner1, corner4);
	
	for (unsigned int y = 0; y < height; ++y)
		for (unsigned int x = 0; x < width; ++x){
			int s = u->find(y * width + x);
			if (set_size.find(s) == set_size.end()) 
				set_size[s] = 0;
			set_size[s] += 1;
		}	
}
					   
int findHole(universe* u, XnPoint3D& pt, unsigned int width, unsigned int height,
			 int& hand, int& background, map<int, int>& set_size, XnPoint3D& hole_center){
	//compute the center of each segment
	map<int, int>::iterator iter;
	map<int, float> mean_x;
	map<int, float> mean_y;
	for(iter = set_size.begin(); iter != set_size.end(); iter++){
		if (iter->first == hand || iter->first == background) continue;
		mean_x[iter->first] = 0;
		mean_y[iter->first] = 0;
	}
	for (unsigned int y = 0; y < height; ++y)
		for (unsigned int x = 0; x < width; ++x){
			int seg = u->find(y * width + x);
			if (seg == hand || seg == background) continue;
			mean_x[seg] += x;
			mean_y[seg] += y;
		}
	for(iter = set_size.begin(); iter != set_size.end(); iter++){
		if (iter->first == hand || iter->first == background) continue;
		mean_x[iter->first] /= float(set_size[iter->first]);
		mean_y[iter->first] /= float(set_size[iter->first]);
	}
	
	//find the non-background segment closest to the hand
	int i_max = -1;
	float maxd = 0.25*cropSize * cropDist / pt.Z;
	int maxpixels = 0;
	for(iter = set_size.begin(); iter != set_size.end(); iter++){
		if (iter->first == hand || iter->first == background) continue;
		float d = sqrt(pow(mean_x[iter->first] - pt.X, 2) +
					   pow(mean_y[iter->first] - pt.Y, 2));

		if (d <= maxd && set_size[iter->first] > maxpixels){
			i_max = iter->first;
			maxpixels = set_size[iter->first];
		}
	}

	if (i_max > -1) {
		hole_center.X = mean_x[i_max], hole_center.Y = mean_y[i_max], hole_center.Z = 0;
	}
	return i_max;
}

// orthorgonal distance regression line
void findPrincipleAxes(universe* u, int width, int height, XnPoint3D& center,
					   int hand_segment, int num_pixels, float** R, float* theta){
	vector<float> diff;
	diff.resize(num_pixels*2);
	int k = 0;
	for(int i = 0; i < width*height; ++i)
		if (u->find(i) == hand_segment){
			int x = i % width;
			int y = i / width;
			diff[k] = x - center.X;
			diff[k+1] = y - center.Y;
			k += 2;
		}
	float numerator = 0;
	float denominator = 0;
	for(k = 0; k < diff.size(); k+=2){
		numerator += diff[k]*diff[k+1];
		denominator += diff[k]*diff[k] - diff[k+1]*diff[k+1];
	}
	float theta1 = M_PI / 4;
	float theta2 = 3 * M_PI / 4;
	if (denominator != 0){
		theta1 = 0.5*atan(2 * numerator / denominator);
		theta2 = theta1 + M_PI / 2;
	}
	
	float d1 = 0;
	float d2 = 0;
	for(k = 0; k < diff.size(); k+=2){
		d1 += pow(-diff[k]*sin(theta1) + diff[k+1]*cos(theta1), 2);
		d2 += pow(-diff[k]*sin(theta2) + diff[k+1]*cos(theta2), 2);
	}
	diff.clear();
	*theta = theta1;
	if (d2 < d1) *theta = theta2;
	R[0][0] = cos(*theta);		R[0][1] = sin(*theta);
	R[1][0] = -sin(*theta);		R[1][1] = cos(*theta);
}

void classifyHandWithHole(XnPoint3D& pt, XnPoint3D& hole, float** R, HandType& hand){
	float hole_y = hole.X * R[1][0] + hole.Y * R[1][1];
	float hand_y = pt.X * R[1][0] + pt.Y * R[1][1];
	if (hole_y < hand_y) hand = LEFT;
	else if (hole_y > hand_y) hand = RIGHT;
	else hand = UNKNOWN;
}

void classifyHandWithoutHole(universe* u, int width, int height, XnPoint3D& pt,
							 int hand_segment, float** R, HandType& hand){
	float min = 0;
	int min_i = -1;
	float max = 0;
	int max_i = -1;
	float cy = pt.X * R[1][0] + pt.Y * R[1][1] ;
	for(int i = 0; i < width*height; ++i)
		if (u->find(i) == hand_segment){
			int x = i % width;
			int y = i / width;
			float new_y = x * R[1][0] + y * R[1][1] - cy;
			if (min_i == -1 || new_y < min){
				min = new_y;
				min_i = i;
			}
			if (max_i == -1 || new_y > max){
				max = new_y;
				max_i = i;
			}
		}
	if (min_i == max_i){
		hand = UNKNOWN;
		return;
	}
	if (fabs(max) > fabs(min)) hand = RIGHT;
	else hand = LEFT;
}

void plane_fitting(vector<XnPoint3D>& points, XnPoint3D& coefficent){
	float A[3][3];
	float b[3];
	for(int i = 0; i < 3; i++){
		for(int j = 0; j < 3; j++) A[i][j] = 0;
		b[i] = 0;
	}
	
	for(int i = 0; i < points.size(); i++){
		A[0][0] += points[i].X*points[i].X;
		A[0][1] += points[i].X*points[i].Y;
		A[0][2] += points[i].X;
		A[1][1] += points[i].Y*points[i].Y;
        A[1][2] += points[i].Y;
		b[0] += points[i].X*points[i].Z;
		b[1] += points[i].Y*points[i].Z;
		b[2] += points[i].Z;
	}
	A[1][0] = A[0][1];
	A[2][0] = A[0][2];
    A[2][1] = A[1][2];
	A[2][2] = points.size();
	
	//compute determinant of A
	float det = A[0][0]*A[1][1]*A[2][2] + A[0][1]*A[1][2]*A[2][0] + 
				A[0][2]*A[1][0]*A[2][1] - A[2][0]*A[1][1]*A[0][2] - 
				A[2][1]*A[1][2]*A[0][0] - A[2][2]*A[1][0]*A[0][1];

	//matrixX contains the equation for the plane.
	if (det == 0){
		coefficent.X = 0;
		coefficent.Y = 0;
		coefficent.Z = 0;
		return;
	}
	
	//now find the inverse of A
	float c[3][3];
	c[0][0] = A[1][1]*A[2][2]-A[2][1]*A[1][2];
	c[0][1] = A[2][0]*A[1][2]-A[1][0]*A[2][2];
	c[0][2] = A[1][0]*A[2][1]-A[2][0]*A[1][1];
	c[1][0] = A[2][1]*A[0][2]-A[0][1]*A[2][2];
	c[1][1] = A[0][0]*A[2][2]-A[2][0]*A[0][2];
	c[1][2] = A[2][0]*A[0][1]-A[0][0]*A[2][1];
	c[2][0] = A[0][1]*A[1][2]-A[1][1]*A[0][2];
	c[2][1] = A[1][0]*A[0][2]-A[0][0]*A[1][2];
	c[2][2] = A[0][0]*A[1][1]-A[1][0]*A[0][1];
	
	//coefficent = (inverse of A)*b
	coefficent.X = (c[0][0]*b[0] + c[0][1]*b[1] + c[0][2]*b[2]) / det;
	coefficent.Y = (c[1][0]*b[0] + c[1][1]*b[1] + c[1][2]*b[2]) / det;
	coefficent.Z = (c[2][0]*b[0] + c[2][1]*b[1] + c[2][2]*b[2]) / det;
	
	if (isnan(coefficent.X) || isnan(coefficent.Y) || isnan(coefficent.Z)){
		coefficent.X = 0;
		coefficent.Y = 0;
		coefficent.Z = 0;
	}	
}

void classifyPinching(universe* u, DepthMetaData& depthMD, unsigned int width, unsigned int height,
					  XnPoint3D& hole_center, int hole_segment, int hand_segment, float palm_width, 
					  float left, float top, FingerType& finger){
	XnPoint3D p;
	vector<XnPoint3D> points;
	float d = MAX_DEPTH;
	for(int i = 0; i < width*height; ++i)
		if (u->find(i) == hand_segment){
			p.X = int(i % width) ;
			p.Y = int(i / width) ;
			p.Z = depthMD(p.X + left, p.Y + top);
			if (p.Z > MAX_DEPTH || p.Z < MIN_DEPTH) continue;
			bool flag = false;
			if (i + 1 < width*height) 
				flag = flag || (u->find(i+1)==hole_segment);
			if (i >= 1) 
				flag = flag || (u->find(i-1)==hole_segment);
			if (i + width < width*height) 
				flag = flag || (u->find(i+width)==hole_segment);
			if (i >= width) 
				flag = flag || (u->find(i-width)==hole_segment);
			
			if (flag) points.push_back(p);
		}
		
	if (points.size() < 3) return;
	XnPoint3D coefficent;
	plane_fitting(points, coefficent);
	if (coefficent.X == 0 && coefficent.Y == 0 && coefficent.Z == 0) return;
	printf("\nCoefficent: (%f, %f, %f)\n", coefficent.X, coefficent.Y, coefficent.Z);
	hole_center.Z = coefficent.X * hole_center.X + coefficent.Y * hole_center.Y + coefficent.Z;
	printf("Center: (%f, %f, %f)\n", hole_center.X, hole_center.Y, hole_center.Z);
	float maxD = 0;
	XnPoint3D maxP;
	for(int i = 0; i < width*height; ++i)
		if (u->find(i) == hand_segment){
			p.X = int(i % width);
			p.Y = int(i / width);
			p.Z = depthMD(p.X + left, p.Y + top);
			if (p.Z > MAX_DEPTH || p.Z < MIN_DEPTH) continue;
			float proj = p.Z - p.X* coefficent.X - p.Y * coefficent.Y;
			float dist = coefficent.Z - proj;
			if (dist > maxD){
				maxD = dist;
				maxP = p;
			}
		}
			
	
	printf("maxD = %f, max point: (%f, %f, %f)\n", maxD, maxP.X, maxP.Y, maxP.Z);
	float ratio = maxD / palm_width;
	printf("ratio = %f\n", ratio);

	if (ratio > 1) return;
	if (ratio >= 0.75) finger = INDEX;
	else if (ratio >= 0.5) finger = MIDDLE;
	else if (ratio >= 0.25) finger = RING;
	else finger = PINKY;
}

void CleanupExit(){
	g_Context.Shutdown();
	fclose(file);
	exit(1);
}

// Callback for when the focus is in progress
void FocusProgress(const XnChar* strFocus, const XnPoint3D& ptPosition,
				   XnFloat fProgress, void* UserCxt){
	printf("Focus progress: %s @(%f,%f,%f): %f\n", strFocus, ptPosition.X,
           ptPosition.Y, ptPosition.Z, fProgress);
}

// callback for session start
void SessionStarting(const XnPoint3D& pt, void* UserCxt){
	printf("Session start: (%f,%f,%f)\n", pt.X, pt.Y, pt.Z);
	g_SessionState = IN_SESSION;
	g_pSessionManager->TrackPoint(pt);
}

// Callback for session end
void SessionEnding(void* UserCxt){
	printf("Session end\n");
	g_SessionState = NOT_IN_SESSION;
}

void NoHands(void* UserCxt){
	if (g_SessionState != NOT_IN_SESSION){
		printf("Quick refocus\n");
		g_SessionState = QUICK_REFOCUS;
	}
}

float computePalmWidth(universe* u, float width, DepthMetaData& depthMD,
					   float left, float top, XnPoint3D& adjustPt, float hand_segment){
	XnPoint3D minX, maxX;
	minX.X = adjustPt.X; minX.Y = adjustPt.Y;
	maxX.X = adjustPt.X; maxX.Y = adjustPt.Y;
	int index = floor(minX.Y)*width + floor(minX.X);
	while(minX.X >= 1){
		if (u->find(index-1) == hand_segment){
			minX.X--;
			index--;
		}
		else break;
	}
	minX.X += left; minX.Y += top;
	minX.Z = depthMD(minX.X, minX.Y);
	index = floor(maxX.Y)*width + floor(maxX.X);
	while(maxX.X + 1 < width){
		if (u->find(index + 1) == hand_segment){ 
			maxX.X++;
			index++;
			
		}
		else break;
	}
	maxX.X += left; maxX.Y += top;
	maxX.Z = depthMD(maxX.X, maxX.Y);
	g_DepthGenerator.ConvertProjectiveToRealWorld(1, &maxX, &maxX);
	g_DepthGenerator.ConvertProjectiveToRealWorld(1, &minX, &minX);
	return sqrt(pow(maxX.X - minX.X, 2) + pow(maxX.Y - minX.Y, 2) + pow(maxX.Z - minX.Z, 2));
}

//Callback for steady hand
void trackHand(XnUInt32 nId){
    XnPoint3D pt;
    DepthMetaData depthMD;
	g_DepthGenerator.GetMetaData(depthMD);
	g_pDrawer->GetLastPosition(nId, pt);
	XnPoint3D center;
	XnUInt32 left, right, top, bottom;
	getBoundingbox(pt, left, right, top, bottom);

	XnPoint3D adjustPt;
	adjustPt.X = pt.X - left;
	adjustPt.Y = pt.Y - top;
	adjustPt.Z = pt.Z;
	if (adjustPt.X < 0 || adjustPt.Y < 0) return;
	
	//hand cropping
	unsigned int width, height;
	image<rgb>* input_img = cropHand(pt, depthMD, left, right, top, bottom, width, height);
	
	if (input_img == NULL) return;
	
	//hole detector
	int num_segments = 0;
	universe* u = get_connected_component(input_img, sigma, k, minSize*pow(cropDist/pt.Z, 2), &num_segments);
	if (u == NULL){
		delete input_img;
		return;
	}
	
	XnPoint3D hole_center;
	map<int, int> set_size;
	findSetSize(u, width, height, set_size);
	num_segments = set_size.size();
	if (num_segments < 2){
		delete input_img;
		delete u;
		return;
	}
	int background_segment = u->find(0);
	int hand_segment = u->find(floor(adjustPt.Y)*width + floor(adjustPt.X));
	if (hand_segment == background_segment){
		delete input_img;
		delete u;
		return;
	}
	float palm_width = g_pDrawer->GetPalmWidth(nId);
	if (palm_width == 0){
		palm_width = computePalmWidth(u, width, depthMD, left, top, adjustPt, hand_segment);
		g_pDrawer->SetPalmWidth(nId, palm_width);
	}
	float theta = 0;
	float** Rot = new float* [2];
	Rot[0] = new float[2]; 
	Rot[1] = new float[2];

	findPrincipleAxes(u, width, height, adjustPt, hand_segment, 
					  set_size[hand_segment], Rot, &theta);
	
	int hole_segment = findHole(u, adjustPt, width, height, hand_segment,
								background_segment, set_size, hole_center);
	HandType hand = g_pDrawer->GetHandType(nId);
	FingerType finger;
	if (hole_segment >= 0){
		printf("Palm width = %f", palm_width);
		classifyHandWithHole(adjustPt, hole_center, Rot, hand);
		center.X = hole_center.X + left;
		center.Y = hole_center.Y + top;
		g_pDrawer->SetHoleCenter(nId, center);
		classifyPinching(u, depthMD, width, height, hole_center, hole_segment, 
						 hand_segment, palm_width, left, top, finger);
		switch (finger){
			case INDEX : printf("INDEX finger, "); break;
			case MIDDLE: printf("MIDDLE finger, "); break;
			case RING : printf("RING finger, "); break;
			case PINKY: printf("PINKY finger, "); break;
			default: printf("UNDEFINED finger, "); break;
		}
#if (USE_HAND_TRACKER == 1)
		if (state == 0) states.push_back(kCGEventLeftMouseDown);
		else states.push_back(kCGEventLeftMouseDragged);
		state = 1;		
#endif
	}
	else{
		printf("NO HOLE\n");
		if (hand == UNKNOWN)
			classifyHandWithoutHole(u, width, height,adjustPt, hand_segment, Rot, hand);
#if (USE_HAND_TRACKER == 1)
		XnPoint3D p;
		p.X = pt.X * widthScale;
		p.Y = pt.Y * heightScale;
		p.Z = 0;
		buffer.push_back(p);
		if (buffer.size() > states.size()){
			if (state == 1) states.push_back(kCGEventLeftMouseUp);
			else states.push_back(kCGEventMouseMoved);
			state = 0;	
		}
#endif
	}
	
	g_pDrawer->SetHandType(nId, hand);
	g_pDrawer->SetFingerType(nId, finger);
	switch (hand){
		case LEFT: printf("LEFT hand\n"); break;
		case RIGHT: printf("RIGHT hand \n"); break;
		default: printf("UNKNOWN hand\n"); break;
	}
	delete [] Rot[0];
	delete [] Rot[1];
	delete [] Rot;
	
#ifdef DEBUG
	counter++;
	image<rgb>* output = color_connected_component(u, width, height,
							hand_segment, background_segment, hole_segment);
	sprintf(filename, "data/segment%d.ppm", counter);
	savePPM(output, filename);
	printf("\tand is written to %s\n", filename);
	delete output;	
#endif
	
	delete input_img;
	delete u;
}

int setup(){
#if (USE_HAND_TRACKER == 1)	
	for(int i = 0 ; i < 640*480; i++) depthbuffer[i] = 0;
#endif
	XnStatus rc = XN_STATUS_OK;
	EnumerationErrors errors;
	
	// Initialize OpenNI
	rc = g_Context.InitFromXmlFile(XML_PATH, &errors);
	CHECK_ERRORS(rc, errors, "InitFromXmlFile");
	CHECK_RC(rc, "InitFromXmlFile");
	
    rc = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
	CHECK_RC(rc, "Find depth generator");
    rc = g_Context.FindExistingNode(XN_NODE_TYPE_HANDS, g_HandsGenerator);
	CHECK_RC(rc, "Find hands generator");
	g_HandsGenerator.SetSmoothing(g_fSmoothing);
	// Create NITE objects
	g_pSessionManager = new XnVSessionManager;
	rc = g_pSessionManager->Initialize(&g_Context,"Click,Wave","RaiseHand",&g_HandsGenerator);
	
	CHECK_RC(rc, "SessionManager::Initialize");
	
	g_pSessionManager->RegisterSession(NULL, SessionStarting, SessionEnding, FocusProgress);
	g_pSessionManager->SetQuickRefocusTimeout(500);
	g_pDrawer = new XnVPointDrawer(historySize, g_DepthGenerator);
    g_pDrawer->SetBoundingBoxSize(cropSize);
    g_pDrawer->SetStandardDistance(cropDist);
    g_pDrawer->SetVerticalBoundingBoxOffset(cropOffset);
    g_pFlowRouter = new XnVFlowRouter;
	g_pFlowRouter->SetActive(g_pDrawer);
	
	g_pSessionManager->AddListener(g_pFlowRouter);
	
	g_pDrawer->RegisterNoPoints(NULL, NoHands);
	g_pDrawer->SetDepthMap(g_bDrawDepthMap);
	g_pDrawer->SetFrameID(g_bPrintFrameID);
	
	// Initialization done. Start generating
	rc = g_Context.StartGeneratingAll();
	CHECK_RC(rc, "StartGenerating");	
	
	return XN_STATUS_OK;
}

#if (USE_HAND_TRACKER == 1)
void update(){
	// Read next available data
	
	g_Context.WaitOneUpdateAll(g_DepthGenerator);
	// Update NITE tree
	g_pSessionManager->Update(&g_Context);	
	DepthMetaData depthMD;
	g_DepthGenerator.GetMetaData(depthMD);
	
	for(int y=0; y < 480; ++y)
		for(int x=0; x<640; ++x)
			depthbuffer[y*480+x] = depthMD(x, y);
}
#endif

#endif
