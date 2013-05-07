#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxOsc.h"

#define HOST "169.254.4.211"
#define PORT 12345

// read from two kinects simultaneously

class testApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
	
	void drawPointCloud0();
	void drawPointCloud1();
	
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	
	ofxKinect kinect0;
	
	ofxKinect kinect1;
	
	ofxCvGrayscaleImage grayImage0; // grayscale depth image
    ofxCvGrayscaleImage backImage0; // grayscale depth image
	
	ofxCvContourFinder contourFinder0;
    float depthMean0;
    ofMatrix4x4 homography0;
    ofPoint src0[4];
        
    ofxCvGrayscaleImage grayImage1; // grayscale depth image
    ofxCvGrayscaleImage backImage1; // grayscale depth image
    
    ofxCvContourFinder contourFinder1;
    float depthMean1;
    ofMatrix4x4 homography1;
    ofPoint src1[4];

    ofPoint dst[4];
    
    ofPoint locutor0;
    ofPoint locutor1;
    ofPoint locutor;
	
	bool bDrawPointCloud;
	
	int nearThreshold;
	int farThreshold;
    int backThreshold;
	
	int angle;
	
	// used for viewing the point cloud
	ofEasyCam easyCam0;
    ofEasyCam easyCam1;
    
    ofxOscSender sender;

    bool newLocutor0;
    bool newLocutor1;
};
