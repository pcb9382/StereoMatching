// Stereo3DReconstruction.cpp : �������̨Ӧ�ó������ڵ㡣
//
#define SIGN(x) ( (x)<0 ? -1:((x)>0?1:0 ) )  

#include <afx.h>
#include <Windows.h>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

//#include "cv.h"
#include "cvaux.h"
//#include "cxcore.h"
//#include "highgui.h"

#include <stdio.h>
#include <iostream>
#include <string.h>

#include <GL/glut.h>

using namespace cv;
using namespace std;

Mat _3dImage;

int width = 0;
int height = 0;
//---OpenGL ȫ�ֱ���  
int glWinWidth = 640, glWinHeight = 480;
double eyex, eyey, eyez, atx, aty, atz;  // eye* - �����λ�ã�at* - ע�ӵ�λ��  
bool leftClickHold = false, rightClickHold = false;
int mx, my;          // ��갴��ʱ�� OpenGL ���ڵ�����  
int ry = 90, rx = 90;    // ��������ע�ӵ�Ĺ۲�Ƕ�  
double radius = 80.0;     // �������ע�ӵ�ľ��� 



int StereoMatch_SGBM(bool bFullDP, Mat &img1, Mat &img2, Rect &roi1, Rect &roi2, int SADWindowSize, int numberOfDisparities, Mat & disp)
{
	StereoSGBM sgbm;

	sgbm.preFilterCap = 63;
	sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;

	int cn = img1.channels();
	sgbm.P1 = 4 * cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
	sgbm.P2 = 32 * cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
	sgbm.minDisparity = 0;
	sgbm.numberOfDisparities = numberOfDisparities;
	sgbm.uniquenessRatio = 10;
	sgbm.speckleWindowSize = 100;
	sgbm.speckleRange = 32;
	sgbm.disp12MaxDiff = 1;
	sgbm.fullDP = bFullDP;

	sgbm(img1, img2, disp);

	return 1;
}




void saveXYZ(const char* filename, const Mat& mat)
{
	const double max_z = 1000;
	FILE* fp = fopen(filename, "wt");
	for (int y = 0; y < mat.rows; y++)
	{
		for (int x = 0; x < mat.cols; x++)
		{
			Vec3f point = mat.at<Vec3f>(y, x);
			if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
			fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
		}
	}
	fclose(fp);
}

void SetupRC()
{
	glClearColor(0.0f, 0.0f, 0.0f, 1);//��RGB(0,0,0)����ɫ�������ɫ������
	glColor3f(1.0f, 0.0f, 0.0f);//��RGB(1,0,0)����ɫ������ͼ��
}
// ��갴����Ӧ����  
void mouse(int button, int state, int x, int y)
{
	if (button == GLUT_LEFT_BUTTON)
	{
		if (state == GLUT_DOWN)
		{
			leftClickHold = true;
		}
		else
		{
			leftClickHold = false;
		}
	}
	if (button == GLUT_RIGHT_BUTTON)
	{
		if (state == GLUT_DOWN)
		{
			rightClickHold = true;
		}
		else
		{
			rightClickHold = false;
		}
	}
}
// ����˶���Ӧ����  
void motion(int x, int y)
{
	int rstep = 10;
	if (leftClickHold == true)
	{
		if (abs(x - mx) > abs(y - my))
		{
			rx -= SIGN(x - mx)*rstep;
		}
		else
		{
			ry += SIGN(y - my)*rstep;
		}

		mx = x;
		my = y;
		glutPostRedisplay();
	}
	if (rightClickHold == true)
	{
		if (y - my > 0)
		{
			radius += 100.0;
		}
		else if (y - my < 0)
		{
			radius -= 100.0;
		}
		radius = std::max(radius, 100.0);
		mx = x;
		my = y;
		glutPostRedisplay();
	}
}
void RenderScene()
{
	// clear screen and depth buffer  
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_PROJECTION);
	// Reset the coordinate system before modifying   
	glLoadIdentity();
	// set the camera position  
	atx = 0.0f;
	aty = 0.0f;
	atz = 100;
	eyex = atx + radius * sin(CV_PI * ry / 180.0f) * cos(CV_PI * rx / 180.0f);
	eyey = aty + radius * cos(CV_PI * ry / 180.0f);
	eyez = atz + radius * sin(CV_PI * ry / 180.0f) * sin(CV_PI * rx / 180.0f);

	glOrtho(-50, 50, -50, 50, 20, 150);
	gluLookAt(eyex, eyey, eyez, atx, aty, atz, 0.0, 1.0, 0.0);

	glPointSize(1.0f);//ָ����Ĵ�С��1�����ص�λ
	glColor3f(1.0f, 0.0f, 0.0f);//ָ�������ɫ
	glBegin(GL_POINTS);//��ʼ����

	int cntPts = 0;
	const double max_z = 1.0e4;
	for (int y = 0; y < _3dImage.rows; y++)
	{
		for (int x = 0; x < _3dImage.cols; x++)
		{
			Vec3f point = _3dImage.at<Vec3f>(y, x);
			if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
			cntPts++;
			glVertex3f(point[0], point[1], point[2]); // ������Ϊ(0,0,0)�ĵط�������һ����
		}
	}
	glColor3f(0.0f, 1.0f, 0.0f);//ָ�������ɫ
	glPointSize(9.0f);//ָ����Ĵ�С��1�����ص�λ
	glVertex3f(0, 0, 0); // ������Ϊ(0,0,0)�ĵط�������һ����

	glEnd();

	// enable blending  
	glEnable(GL_BLEND);
	// enable read-only depth buffer  
	glDepthMask(GL_FALSE);
	// set the blend function to what we use for transparency  
	glBlendFunc(GL_SRC_ALPHA, GL_ONE);
	// set back to normal depth buffer mode (writable)  
	glDepthMask(GL_TRUE);
	// disable blending  
	glDisable(GL_BLEND);

	glFlush();
	glutSwapBuffers();
}



int _tmain(int argc, _TCHAR* argv[])
{
	string strWorkPath = "G:\\Project1\\Project1\\";
	//string strImgFullPath1 = strWorkPath + "left1.jpg";
	//string strImgFullPath2 = strWorkPath + "right1.jpg";
	//string strIntrinsicFullPath = strWorkPath + "intrinsic.xml";
	//string strExtrinsicFullPath = strWorkPath + "extrinsic.xml";
	string strDisparityImgFullPath = strWorkPath + "disparity.bmp";
	string strPointCloudFullPath = strWorkPath +   "PC.txt";
	IplImage * img1 = cvLoadImage("img1r.jpg", 0);
	IplImage * img2 = cvLoadImage("img2r.jpg", 0);
	CvMat *Q = (CvMat*)cvLoad("Q.xml");
	Mat _Q(Q);

	enum { STEREO_BM = 0, STEREO_SGBM = 1, STEREO_HH = 2, STEREO_VAR = 3, STEREO_GC = 4 };
	int alg = STEREO_SGBM;
	///SADWindowSize��������Ϊ����1������
	int SADWindowSize = 0;
	int numberOfDisparities = 0;
	
	 if (alg == STEREO_SGBM)
	{
		SADWindowSize = 15;
		numberOfDisparities = 256;
	}
	
	bool no_display = false;
	float scale = 1.f;

	int color_mode = alg == STEREO_BM ? 0 : -1;
	
   

	Rect roi1, roi2;
	

	
	

	


	int64 t = getTickCount();
	//�����㷨��������ƥ��
	Mat disp, disp8, disp_R;
	
	 if (alg == STEREO_SGBM)
		 StereoMatch_SGBM(false, (Mat)img1, (Mat)img2, roi1, roi2, SADWindowSize, numberOfDisparities, disp);
 
	
	t = getTickCount() - t;
	printf("Time elapsed: %fms\n", t * 1000 / getTickFrequency());

	//ת���Ӳ�ͼ
	if (alg != STEREO_VAR)
		disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));
	else
		disp.convertTo(disp8, CV_8U);

	namedWindow("disparity", 0);
	imshow("disparity", disp8);
	//cvWaitKey(0);
	fflush(stdout);
	printf("\n");
	//���Ӳ�ͼд���ļ�
	imwrite(strDisparityImgFullPath, disp8);
	fflush(stdout);
	//���Ӳ�ͼת��Ϊ��ά���Ʋ�д���ļ�
	reprojectImageTo3D(disp, _3dImage, _Q, true);
	IplImage pImg1 = IplImage(_3dImage); 
	CvMat mapMat = _3dImage;
	cvSave("map3D.xml", &mapMat);
	printf("storing the point cloud...");
	printf("\n");
	saveXYZ(strPointCloudFullPath.c_str(), _3dImage);
	printf("press any key to continue...");

	////------------------ʹ��OpenGL��ʾ��ά����-------------------------
	if (!no_display)
	{
		glutInit(&argc, argv);
		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
		glutInitWindowSize(800, 600);
		glutInitWindowPosition(10, 10);
		glutCreateWindow("PointCloud");
		glutDisplayFunc(RenderScene);

		//glutKeyboardFunc(myKeyboard);
		glutMouseFunc(mouse);
		glutMotionFunc(motion);

		SetupRC();
		glutMainLoop();
	}




	//cvWaitKey(0);

	return 0;
}
