#ifndef XNV_POINT_DRAWER_H_
#define XNV_POINT_DRAWER_H_
#define MAX_DEPTH 2000
#define MIN_DEPTH 200
#include <map>
#include <list>
#include <XnCppWrapper.h>
#include <XnVPointControl.h>

#include "XnVSessionManager.h"

typedef enum
{
	IN_SESSION,
	NOT_IN_SESSION,
	QUICK_REFOCUS
} SessionState;

//hand and fingers
typedef enum {UNKNOWN, LEFT, RIGHT} HandType;
typedef enum {UNDEFINED,INDEX,MIDDLE,RING,PINKY,THUMB} FingerType;

void PrintSessionState(SessionState eState);
/**
 * This is a point control, which stores the history of every point
 * It can draw all the points as well as the depth map.
 */

void trackHand(XnUInt32 nId);

class XnVPointDrawer : public XnVPointControl
{
public:
	XnVPointDrawer(XnUInt32 nHistorySize, xn::DepthGenerator depthGenerator);
	virtual ~XnVPointDrawer();

	/**
	 * Handle a new message.
	 * Calls other callbacks for each point, then draw the depth map 
     * (if needed) and the points
	 */
	void Update(XnVMessage* pMessage);

	/**
	 * Handle creation of a new point
	 */
	void OnPointCreate(const XnVHandPointContext* cxt);
	/**
	 * Handle new position of an existing point
	 */
	void OnPointUpdate(const XnVHandPointContext* cxt);
	/**
	 * Handle destruction of an existing point
	 */
	void OnPointDestroy(XnUInt32 nID);

	/**
	 * Draw the points, each with its own color.
	 */
	void Draw() const;

	/**
	 * Change mode - should draw the depth map?
	 */
	void SetDepthMap(XnBool bDrawDM);

	/**
	 * Change mode - print out the frame id
	 */ 
	void SetFrameID(XnBool bFrameID);
	
	/**
	 * Change mode - set hand bounding box size
	 */ 
	void SetBoundingBoxSize(XnFloat boxSize);
	
	/**
     * Change mode - set standard distance of the hand to kinect
     */
    void SetStandardDistance(XnFloat dist);
    
	/**
     * Change mode - set vertical bounding box offset
     */
    void SetVerticalBoundingBoxOffset(XnFloat offset);
  
	/**
	 * Set the center of the hole
	 */
	void SetHoleCenter(XnUInt32 nId, XnPoint3D& center);
	
	/**
	 * Set the hand type
	 */
	void SetHandType(XnUInt32 nId, HandType h);
	
	/**
	 * Set the finger type
	 */
	void SetFingerType(XnUInt32 nId, FingerType f);
	
	/**
	 * Get history of a point
	 */ 
	void GetHistory(XnUInt32 nId, std::list<XnPoint3D>& history);
	
	/**
	 * Get last position of a point
	 */
	void GetLastPosition(XnUInt32 nId, XnPoint3D& pt);
	
	/**
	 * Get the center of the hole
	 */
	XnPoint3D GetHoleCenter(XnUInt32 nId);
	
	/**
	 * Set the hand type
	 */
	HandType GetHandType(XnUInt32 nId);
	
	/**
	 * Set the finger type
	 */
	FingerType GetFingerType(XnUInt32 nId);
	
	void SetPalmWidth(XnUInt32 nId, float w) { m_width[nId] = w;}
	
	float GetPalmWidth(XnUInt32 nId){ return m_width[nId]; }

	
protected:
	// Number of previous position to store for each hand
	XnUInt32 m_nHistorySize;
	// previous positions per hand
	std::map<XnUInt32, std::list<XnPoint3D> > m_History;
	// current position of the hole
	std::map<XnUInt32, XnPoint3D> m_hole;
	//curent hand type
	std::map<XnUInt32, HandType> m_hand;
	//current finger type
	std::map<XnUInt32, FingerType> m_finger;
	//current palm-width
	std::map<XnUInt32, float> m_width;
	
	
	// Source of the depth map
	xn::DepthGenerator m_DepthGenerator;
	
	XnFloat* m_pfPositionBuffer;

	XnBool m_bDrawDM;
	XnBool m_bFrameID;
	
	XnFloat m_box_size;
    XnFloat m_distance;
    XnFloat m_offset;
};

#endif
