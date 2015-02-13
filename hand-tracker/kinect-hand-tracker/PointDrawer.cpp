#include "PointDrawer.h"
#include "XnVDepthMessage.h"
#include <XnVHandPointContext.h>

#if (XN_PLATFORM == XN_PLATFORM_MACOSX)
	#include <GLUT/glut.h>
#else
	#include <GL/glut.h>
#endif


// Constructor. Receives the number of previous positions to store per hand,
// and a source for depth map
XnVPointDrawer::XnVPointDrawer(XnUInt32 nHistory, 
                               xn::DepthGenerator depthGenerator) :
	XnVPointControl("XnVPointDrawer"),
	m_nHistorySize(nHistory), 
    m_DepthGenerator(depthGenerator),
    m_bDrawDM(false), m_bFrameID(false), 
    m_box_size(100.0f), m_distance(100.0f), m_offset(10.0f)
{
	m_pfPositionBuffer = new XnFloat[nHistory*3];
}

// Destructor. Clear all data structures
XnVPointDrawer::~XnVPointDrawer()
{
	std::map<XnUInt32, std::list<XnPoint3D> >::iterator iter;
	for (iter = m_History.begin(); iter != m_History.end(); ++iter)
	{
		iter->second.clear();
		
	}
	m_History.clear();
	m_hole.clear();
	m_hand.clear();
	m_finger.clear();
	delete [] m_pfPositionBuffer;
}

// Change whether or not to draw the depth map
void XnVPointDrawer::SetDepthMap(XnBool bDrawDM)
{
	m_bDrawDM = bDrawDM;
}
// Change whether or not to print the frame ID
void XnVPointDrawer::SetFrameID(XnBool bFrameID)
{
	m_bFrameID = bFrameID;
}
// Change bounding box size
void XnVPointDrawer::SetBoundingBoxSize(XnFloat boxSize)
{
	m_box_size = boxSize;
}
// Change standard distance to Kinect
void XnVPointDrawer::SetStandardDistance(XnFloat dist)
{
	m_distance = dist;
}
// Change vertical bounding box offset
void XnVPointDrawer::SetVerticalBoundingBoxOffset(XnFloat offset)
{
    m_offset = offset;
}

// Set the hole center
void XnVPointDrawer::SetHoleCenter(XnUInt32 nId, XnPoint3D& center){
	if (m_hole.find(nId) != m_hole.end()){
		m_hole[nId].X = center.X;
		m_hole[nId].Y = center.Y;
		m_hole[nId].Z = 0;
	}
}

//Set the hand type
void XnVPointDrawer::SetHandType(XnUInt32 nId, HandType h){
	if (m_hand.find(nId) != m_hand.end())
		m_hand[nId] = h;
}

//Set the finger type
void XnVPointDrawer::SetFingerType(XnUInt32 nId, FingerType f){
	if (m_finger.find(nId) != m_finger.end())
		m_finger[nId] = f;
}

// Get history of a point
void XnVPointDrawer::GetHistory(XnUInt32 nId, std::list<XnPoint3D>& history){
	history = m_History[nId];
}

// Get last position of a point
void XnVPointDrawer::GetLastPosition(XnUInt32 nId, XnPoint3D& pt){
	pt = m_History[nId].back();
}

//Get the center of the hole
XnPoint3D XnVPointDrawer::GetHoleCenter(XnUInt32 nId){
	return m_hole[nId];
}

//Set the hand type
HandType XnVPointDrawer::GetHandType(XnUInt32 nId){
	return m_hand[nId];
}

//Set the finger type
FingerType XnVPointDrawer::GetFingerType(XnUInt32 nId){
	return m_finger[nId];
}

// Handle creation of a new hand
static XnBool bShouldPrint = false;
void XnVPointDrawer::OnPointCreate(const XnVHandPointContext* cxt)
{
	printf("** %d\n", cxt->nID);
	// Create entry for the hand
	m_History[cxt->nID].clear();
	XnPoint3D hole;
	hole.X = -1;
	hole.Y = -1;
	hole.Z = 0;
	m_hole[cxt->nID] = hole;
	m_hand[cxt->nID] = UNKNOWN;
	m_finger[cxt->nID] = UNDEFINED;
	m_width[cxt->nID] = 0;
	bShouldPrint = true;
	OnPointUpdate(cxt);
	bShouldPrint = true;
}
// Handle new position of an existing hand
void XnVPointDrawer::OnPointUpdate(const XnVHandPointContext* cxt)
{
	XnPoint3D hole;
	hole.X = -1;
	hole.Y = -1;
	hole.Z = 0;
	m_hole[cxt->nID] = hole;
	// positions are kept in projective coordinates, since they are only used for drawing
	XnPoint3D ptProjective(cxt->ptPosition);
	if (bShouldPrint)printf("Point (%f,%f,%f)", ptProjective.X, ptProjective.Y,
                                                ptProjective.Z);
	m_DepthGenerator.ConvertRealWorldToProjective(1, &ptProjective,
                                                  &ptProjective);
	if (bShouldPrint)printf(" -> (%f,%f,%f)\n", ptProjective.X, ptProjective.Y,
                                                ptProjective.Z);
	
	if (ptProjective.Z < MIN_DEPTH || ptProjective.Z > MAX_DEPTH){
		return;
	}
	
	// Add new position to the history buffer
	m_History[cxt->nID].push_front(ptProjective);
	trackHand(cxt->nID);
    // Keep size of history buffer
	if (m_History[cxt->nID].size() > m_nHistorySize)
		m_History[cxt->nID].pop_back();
	bShouldPrint = false;
}

// Handle destruction of an existing hand
void XnVPointDrawer::OnPointDestroy(XnUInt32 nID)
{
	// No need for the history buffer
	m_History.erase(nID);
	m_hole.erase(nID);
	m_hand.erase(nID);
	m_finger.erase(nID);
	m_width.erase(nID);
}

float g_pDepthHist[MAX_DEPTH];
unsigned int getClosestPowerOfTwo(unsigned int n)
{
	unsigned int m = 2;
	while(m < n) m<<=1;

	return m;
}
GLuint initTexture(void** buf, int& width, int& height)
{
	GLuint texID = 0;
	glGenTextures(1,&texID);

	width = getClosestPowerOfTwo(width);
	height = getClosestPowerOfTwo(height); 
	*buf = new unsigned char[width*height*4];
	glBindTexture(GL_TEXTURE_2D,texID);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	return texID;
}

GLfloat texcoords[8];
void DrawRectangle(float topLeftX, float topLeftY,
                   float bottomRightX, float bottomRightY)
{
	GLfloat verts[10] = {topLeftX, topLeftY, topLeftX, bottomRightY,
		                 bottomRightX, bottomRightY, bottomRightX, topLeftY};
	glVertexPointer(2, GL_FLOAT, 0, verts);
	glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

	glFlush();
}
void DrawTexture(float topLeftX, float topLeftY,
                 float bottomRightX, float bottomRightY)
{
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glTexCoordPointer(2, GL_FLOAT, 0, texcoords);

	DrawRectangle(topLeftX, topLeftY, bottomRightX, bottomRightY);

	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
}

void DrawDepthMap(const xn::DepthMetaData& dm)
{
	static bool bInitialized = false;	
	static GLuint depthTexID;
	static unsigned char* pDepthTexBuf;
	static int texWidth, texHeight;

	float topLeftX;
	float topLeftY;
	float bottomRightY;
	float bottomRightX;
	float texXpos;
	float texYpos;

	if(!bInitialized)
	{
		XnUInt16 nXRes = dm.XRes();
		XnUInt16 nYRes = dm.YRes();
		texWidth =  getClosestPowerOfTwo(nXRes);
		texHeight = getClosestPowerOfTwo(nYRes);
        printf("text width = %d, text height = %d\n", texWidth, texHeight);
		depthTexID = initTexture((void**)&pDepthTexBuf,texWidth, texHeight) ;

		bInitialized = true;

		topLeftX = nXRes;
		topLeftY = 0;
		bottomRightY = nYRes;
		bottomRightX = 0;
		texXpos =(float)nXRes/texWidth;
		texYpos  =(float)nYRes/texHeight;

		memset(texcoords, 0, 8*sizeof(float));
		texcoords[0] = texXpos, texcoords[1] = texYpos, texcoords[2] = texXpos, texcoords[7] = texYpos;

	}
	unsigned int nValue = 0;
	unsigned int nHistValue = 0;
	unsigned int nIndex = 0;
	unsigned int nX = 0;
	unsigned int nY = 0;
	unsigned int nNumberOfPoints = 0;
	XnUInt16 g_nXRes = dm.XRes();
	XnUInt16 g_nYRes = dm.YRes();

	unsigned char* pDestImage = pDepthTexBuf;

	const XnUInt16* pDepth = dm.Data();

	// Calculate the accumulative histogram
	memset(g_pDepthHist, 0, MAX_DEPTH*sizeof(float));
	for (nY=0; nY<g_nYRes; nY++)
	{
		for (nX=0; nX<g_nXRes; nX++)
		{
			nValue = *pDepth;

			if (nValue != 0 && nValue < MAX_DEPTH && nValue > MIN_DEPTH)
			{
				g_pDepthHist[nValue]++;
				nNumberOfPoints++;
			}

			pDepth++;
		}
	}

	for (nIndex=1; nIndex<MAX_DEPTH; nIndex++)
	{
		g_pDepthHist[nIndex] += g_pDepthHist[nIndex-1];
	}
	if (nNumberOfPoints)
	{
		for (nIndex=1; nIndex<MAX_DEPTH; nIndex++)
		{
			g_pDepthHist[nIndex] = (unsigned int)(256 * (1.0f - (g_pDepthHist[nIndex] / nNumberOfPoints)));
		}
	}

	pDepth = dm.Data();
	{
		XnUInt32 nIndex = 0;
		// Prepare the texture map
		for (nY=0; nY<g_nYRes; nY++)
		{
			for (nX=0; nX < g_nXRes; nX++, nIndex++)
			{
				nValue = *pDepth;

				if (nValue != 0 && nValue < MAX_DEPTH && nValue > MIN_DEPTH)
				{
					nHistValue = g_pDepthHist[nValue];

					pDestImage[0] = nHistValue;
					pDestImage[1] = nHistValue;
					pDestImage[2] = nHistValue;
				}
				else
				{
					pDestImage[0] = 0;
					pDestImage[1] = 0;
					pDestImage[2] = 0;
				}

				pDepth++;
				pDestImage+=3;
			}

			pDestImage += (texWidth - g_nXRes) *3;
		}
	}
	glBindTexture(GL_TEXTURE_2D, depthTexID);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texWidth, texHeight, 0, 
                 GL_RGB, GL_UNSIGNED_BYTE, pDepthTexBuf);

	// Display the OpenGL texture map
	glColor4f(0.5,0.5,0.5,1);

	glEnable(GL_TEXTURE_2D);
	DrawTexture(dm.XRes(),dm.YRes(),0,0);	
	glDisable(GL_TEXTURE_2D);
}

void glPrintString(void *font, char *str)
{
	int i,l = strlen(str);

	for(i=0; i<l; i++)
	{
		glutBitmapCharacter(font,*str++);
	}
}
void DrawFrameID(XnUInt32 nFrameID)
{
	glColor4f(1,0,0,1);
	glRasterPos2i(20, 50);
	XnChar strLabel[20];
	sprintf(strLabel, "%d", nFrameID);
	glPrintString(GLUT_BITMAP_HELVETICA_18, strLabel);
}

// Colors for the points
XnFloat Colors[][3] =
{
	{1,0,0},	// Red. Reserved for primary point
	{0,1,0},	// Green
	{0,0.5,1},	// Light blue
	{1,1,0},	// Yellow
	{1,0.5,0},	// Orange
	{1,0,1},	// Purple
	{1,1,1}		// White
};
XnUInt32 nColors = 6;

void XnVPointDrawer::Draw() const
{
	std::map<XnUInt32, std::list<XnPoint3D> >::const_iterator PointIterator;
	std::map<XnUInt32, XnPoint3D>::const_iterator holeIterator;
	holeIterator = m_hole.begin();
	// Go over each existing hand
	for (PointIterator = m_History.begin();
		PointIterator != m_History.end();
		++PointIterator, ++holeIterator)
	{
		// Clear buffer
		XnUInt32 nPoints = 0;
		XnUInt32 i = 0;
		XnUInt32 Id = PointIterator->first;
        XnFloat x, y;
        XnFloat z = 0;
		// Go over all previous positions of current hand
		std::list<XnPoint3D>::const_iterator PositionIterator;
		for (PositionIterator = PointIterator->second.begin();
			PositionIterator != PointIterator->second.end();
			++PositionIterator, ++i)
		{
			// Add position to buffer
			XnPoint3D pt(*PositionIterator);
			m_pfPositionBuffer[3*i] = pt.X;
			m_pfPositionBuffer[3*i + 1] = pt.Y;
			m_pfPositionBuffer[3*i + 2] = 0;//pt.Z();
            z = pt.Z;
		}
		// Set color
		XnUInt32 nColor = Id % nColors;
		XnUInt32 nSingle = GetPrimaryID();
		if (Id == GetPrimaryID())
			nColor = 0;
		// Draw buffer:
		glColor4f(Colors[nColor][0], Colors[nColor][1], Colors[nColor][2], 1.0f);
		glPointSize(2);
		glVertexPointer(3, GL_FLOAT, 0, m_pfPositionBuffer);
		glDrawArrays(GL_LINE_STRIP, 0, i);
		glPointSize(4);
		glDrawArrays(GL_POINTS, 0, 1);
		if (i > 0 && z != 0){
			x = m_pfPositionBuffer[3*i - 3];
			y = m_pfPositionBuffer[3*i - 2];
			XnFloat size = m_box_size * m_distance / z;
			XnFloat offset = m_offset * m_distance / z;
			XnFloat points[15] = { x - size, y - size, 0,
								x - size, y + size + offset, 0,
								x + size, y + size + offset, 0,
								x + size, y - size, 0,
								x - size, y - size, 0 };
			glPointSize(2);
			glVertexPointer(3, GL_FLOAT, 0, points);
			glDrawArrays(GL_LINE_STRIP, 0, 5);
		}
		
		XnPoint3D hole = holeIterator->second;
		if (hole.X != -1 && hole.Y != -1){
			XnFloat h[3] = {hole.X, hole.Y, hole.Z};
			glVertexPointer(3, GL_FLOAT, 0, h);
			glPointSize(8);
			glDrawArrays(GL_POINTS, 0, 1);
		}
		
		glFlush();
	}
}

// Handle a new Message
void XnVPointDrawer::Update(XnVMessage* pMessage)
{
	// PointControl's Update calls all callbacks for each hand
	XnVPointControl::Update(pMessage);
#if (USE_HAND_TRACKER == 0)
	if (m_bDrawDM)
	{
		// Draw depth map
		xn::DepthMetaData depthMD;
		m_DepthGenerator.GetMetaData(depthMD);
		DrawDepthMap(depthMD);
	}
	if (m_bFrameID)
	{
		// Print out frame ID
		xn::DepthMetaData depthMD;
		m_DepthGenerator.GetMetaData(depthMD);
		DrawFrameID(depthMD.FrameID());
	}
	
	Draw();
#endif
}

void PrintSessionState(SessionState eState)
{
	glColor4f(1,0,1,1);
	glRasterPos2i(20, 20);
	
	XnChar strLabel[200];

	switch (eState)
	{
	    case IN_SESSION:
		    sprintf(strLabel, "Tracking hands"); break;
	    case NOT_IN_SESSION:
		    sprintf(strLabel, "Perform click or wave gestures to track hand"); break;
	    case QUICK_REFOCUS:
		    sprintf(strLabel, "Raise your hand for it to be identified, or perform click or wave gestures"); break;
	}
	glPrintString(GLUT_BITMAP_HELVETICA_18, strLabel);
}
