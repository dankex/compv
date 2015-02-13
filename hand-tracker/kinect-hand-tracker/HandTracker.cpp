/*******************************************************************************
*                                                                              *
*   PrimeSense NITE 1.3 - Point Viewer Sample                                  *
*   Copyright (C) 2010 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/
#define USE_HAND_TRACKER 0
#include "HandTracker.h"

// this function is called each frame
void glutDisplay (void)
{
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	// Setup the OpenGL viewpoint
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	
	DepthMetaData depthMD;
	g_DepthGenerator.GetMetaData(depthMD);
	glOrtho(0, depthMD.XRes(), depthMD.YRes(), 0, -1.0, 1.0);
	
	glDisable(GL_TEXTURE_2D);
	
	if (!g_bPause)
	{
		// Read next available data
		g_Context.WaitOneUpdateAll(g_DepthGenerator);
		// Update NITE tree
		g_pSessionManager->Update(&g_Context);
		
		PrintSessionState(g_SessionState);
	}
	
	glutSwapBuffers();
	
}

void glutIdle (void)
{
	if (g_bQuit) {
		CleanupExit();
	}
	
	// Display the frame
	glutPostRedisplay();
}

void glutKeyboard (unsigned char key, int x, int y)
{
	switch (key)
	{
		case 27:
			// Exit
			CleanupExit();
		case'p':
			// Toggle pause
			g_bPause = !g_bPause;
			break;
		case 'd':
			// Toggle drawing of the depth map
			g_bDrawDepthMap = !g_bDrawDepthMap;
			g_pDrawer->SetDepthMap(g_bDrawDepthMap);
			break;
		case 'f':
			g_bPrintFrameID = !g_bPrintFrameID;
			g_pDrawer->SetFrameID(g_bPrintFrameID);
			break;
		case 's':
			// Toggle smoothing
			if (g_fSmoothing == 0)
				g_fSmoothing = 0.1;
			else 
				g_fSmoothing = 0;
			g_HandsGenerator.SetSmoothing(g_fSmoothing);
			break;
		case 'e':
			// end current session
			g_pSessionManager->EndSession();
			break;
	}
}
void glInit (int * pargc, char ** argv)
{
	glutInit(pargc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	glutCreateWindow ("Hand Tracker");
	//glutFullScreen();
	glutSetCursor(GLUT_CURSOR_NONE);
	
	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);
	
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);
	
	glEnableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
}

int main(int argc, char ** argv)
{
	setup();
	
	// Mainloop
	glInit(&argc, argv);
	glutMainLoop();

}
