#include "testApp.h"

#include "homography.h"

//--------------------------------------------------------------
void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect0.setRegistration(true);
    
	kinect0.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	
	kinect0.open(1);		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
	
    kinect1.setRegistration(true);
	kinect1.init();
	kinect1.open(0);
	
	grayImage0.allocate(kinect0.width, kinect0.height);
    
    grayImage1.allocate(kinect1.width, kinect1.height);
    
    backImage0.allocate(kinect0.width, kinect0.height);
    
    backImage1.allocate(kinect1.width, kinect1.height);
	
	nearThreshold = 255;
	farThreshold = 1;
    backThreshold = 10;
	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 21;
	kinect0.setCameraTiltAngle(angle);
    kinect1.setCameraTiltAngle(angle);
	
	// start from the front
	bDrawPointCloud = false;
    
    src0[0].set(0,0);
    src0[1].set(0,1);
    src0[2].set(1,1);
    src0[3].set(1,0);

    src1[0].set(0,0);
    src1[1].set(0,1);
    src1[2].set(1,1);
    src1[3].set(1,0);
    
    dst[0].set(0,0);
    dst[1].set(0,1);
    dst[2].set(1,1);
    dst[3].set(1,0);
    
    homography0=findHomography(src0, dst);
    homography1=findHomography(src1, dst);
    
    locutor0.set(0,0);
    locutor1.set(0,0);
    locutor.set(0,0);
    
    // open an outgoing connection to HOST:PORT
	sender.setup(HOST, PORT);
    
}

//--------------------------------------------------------------
void testApp::update() {
	
	ofBackground(100, 100, 100);
	
	kinect0.update();
	
	// there is a new frame and we are connected
	if(kinect0.isFrameNew()) {
		
		// load grayscale depth image from the kinect source
		grayImage0.setFromPixels(kinect0.getDepthPixels(), kinect0.width, kinect0.height);
		
			// or we do it ourselves - show people how they can work with the pixels
			unsigned char * pix = grayImage0.getPixels();
            unsigned char * bpix = backImage0.getPixels();
            int n=1;
            depthMean0=0.;
			int numPixels = grayImage0.getWidth() * grayImage0.getHeight();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold && abs(pix[i]-bpix[i])>backThreshold) {
                    n++;
                    depthMean0+=pix[i];
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
        depthMean0/=n;
		
		// update the cv images
		grayImage0.flagImageChanged();
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder0.findContours(grayImage0, 2000, (kinect0.width*kinect0.height)/2, 1, false);
        
        if(contourFinder0.nBlobs){
            
            ofPoint blob=ofPoint(contourFinder0.blobs[0].centroid.x,depthMean0);
            
            if(ofGetKeyPressed('a')){
                if(ofGetKeyPressed('1')){
                    src0[0].set(blob);
                    homography0=findHomography(src0, dst);
                }
                else if(ofGetKeyPressed('2')){
                    src0[1].set(blob);
                    homography0=findHomography(src0, dst);
                }
                else if(ofGetKeyPressed('3')){
                    src0[2].set(blob);
                    homography0=findHomography(src0, dst);
                }
                else if(ofGetKeyPressed('4')){
                    src0[3].set(blob);
                    homography0=findHomography(src0, dst);
                }
            }
               
               ofPoint l=homography0*blob;
            
            if(l.x>=0. && l.x<=1.){// && l.y>=0. && l.y<=1.){
                locutor0.set(l/2.);
                newLocutor0=true;
            }
            else
                newLocutor0=false;

        }
	}
    
    kinect1.update();
	
	// there is a new frame and we are connected
	if(kinect1.isFrameNew()) {
		
		// load grayscale depth image from the kinect source
		grayImage1.setFromPixels(kinect1.getDepthPixels(), kinect1.width, kinect1.height);
		
            // or we do it ourselves - show people how they can work with the pixels
            unsigned char * pix = grayImage1.getPixels();
            unsigned char * bpix = backImage1.getPixels();
            int n=1;
            depthMean1=0.;
            int numPixels = grayImage1.getWidth() * grayImage1.getHeight();
            for(int i = 0; i < numPixels; i++) {
                if(pix[i] < nearThreshold && pix[i] > farThreshold && abs(pix[i]-bpix[i])>backThreshold) {
                    n++;
                    depthMean1+=pix[i];
                    pix[i] = 255;
                } else {
                    pix[i] = 0;
                }
            }
            
        depthMean1/=n;
		
		// update the cv images
		grayImage1.flagImageChanged();
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder1.findContours(grayImage1, 2000, (kinect1.width*kinect1.height)/2, 1, false);
        
        if(contourFinder1.nBlobs){
            
            ofPoint blob=ofPoint(contourFinder1.blobs[0].centroid.x,depthMean1);
            
            if(ofGetKeyPressed('l')){
                if(ofGetKeyPressed('1')){
                    src1[0].set(blob);
                    homography1=findHomography(src1, dst);
                }
                else if(ofGetKeyPressed('2')){
                    src1[1].set(blob);
                    homography1=findHomography(src1, dst);
                }
                else if(ofGetKeyPressed('3')){
                    src1[2].set(blob);
                    homography1=findHomography(src1, dst);
                }
                else if(ofGetKeyPressed('4')){
                    src1[3].set(blob);
                    homography1=findHomography(src1, dst);
                }
            }
            
            ofPoint l=homography1*blob;
            
            if(l.x>=0. && l.x<=1. ){//&& l.y>=0. && l.y<=1.){
                locutor1.set((l.x+1.)/2.,l.y);
                newLocutor1=true;
            }
            else
                newLocutor1=false;
            
        }
	}
    
    if((ofGetFrameNum()%10)==0){
        if(newLocutor1 || newLocutor0){
            
            if(newLocutor1 && newLocutor0){
                locutor.set((locutor0+locutor1)/2.);
            }
            else if(newLocutor0){
                locutor.set(locutor0);
            }
            else if(newLocutor1){
                locutor.set(locutor1);
            }
            
            ofxOscMessage m;
            m.setAddress("/locutor");
            m.addFloatArg(locutor.x);
            sender.sendMessage(m);
        }
    }
	
}

//--------------------------------------------------------------
void testApp::draw() {
    
    ofPushMatrix();
    
    ofTranslate(0,0);
    
	ofSetColor(255, 255, 255);
	
	if(bDrawPointCloud) {
		easyCam0.begin();
		drawPointCloud0();
		easyCam0.end();
	} else {
		// draw from the live kinect
		kinect0.drawDepth(10, 10, 400, 300);
		kinect0.draw(420, 10, 400, 300);
		
		grayImage0.draw(10, 320, 400, 300);
		contourFinder0.draw(10, 320, 400, 300);
        
        backImage0.draw(420,320,400,300);
        
        ofSetColor(255,0,0);
        ofCircle(420+400*(2*locutor0.x),320+150,10);
		
    }
	
	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;
	reportStream << "accel is: " << ofToString(kinect0.getMksAccel().x, 2) << " / "
	<< ofToString(kinect0.getMksAccel().y, 2) << " / "
	<< ofToString(kinect0.getMksAccel().z, 2) << endl
	<< "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
	<< "set near threshold " << nearThreshold << " (press: + -)" << endl
	<< "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder0.nBlobs
	<< ", fps: " << ofGetFrameRate() << endl
	<< "press c to close the connection and o to open it again, connection is: " << kinect0.isConnected() << endl
	<< "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl;
	ofDrawBitmapString(reportStream.str(),20,652);
    
    ofPopMatrix();
    
    ofPushMatrix();
    
    ofTranslate(900,0);
    
	ofSetColor(255, 255, 255);
	
	if(bDrawPointCloud) {
		easyCam1.begin();
		drawPointCloud1();
		easyCam1.end();
	} else {
		// draw from the live kinect
		kinect1.drawDepth(10, 10, 400, 300);
		kinect1.draw(420, 10, 400, 300);
		
		grayImage1.draw(10, 320, 400, 300);
		contourFinder1.draw(10, 320, 400, 300);
        
        backImage1.draw(420,320,400,300);
        ofSetColor(255,0,0);
        ofCircle(420+400*(2*(locutor1.x-0.5)),320+150,10);
		
    }
	
	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream1;
	reportStream1 << "accel is: " << ofToString(kinect1.getMksAccel().x, 2) << " / "
	<< ofToString(kinect1.getMksAccel().y, 2) << " / "
	<< ofToString(kinect1.getMksAccel().z, 2) << endl
	<< "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
	<< "set near threshold " << nearThreshold << " (press: + -)" << endl
	<< "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder1.nBlobs
	<< ", fps: " << ofGetFrameRate() << endl
	<< "press c to close the connection and o to open it again, connection is: " << kinect1.isConnected() << endl
	<< "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl;
	ofDrawBitmapString(reportStream1.str(),20,652);
    
    ofPopMatrix();
    
    ofSetColor(0);
    ofRect(0,0,ofGetWidth(),20);
    ofSetColor(0,255,0);
    ofCircle(locutor.x*ofGetWidth(),10,20);
}

void testApp::drawPointCloud0() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect0.getDistanceAt(x, y) > 0) {
				mesh.addColor(kinect0.getColorAt(x,y));
				mesh.addVertex(kinect0.getWorldCoordinateAt(x, y));
			}
		}
	}
	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	glEnable(GL_DEPTH_TEST);
	mesh.drawVertices();
	glDisable(GL_DEPTH_TEST);
	ofPopMatrix();
}

void testApp::drawPointCloud1() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 1;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect1.getDistanceAt(x, y) > 0) {
				mesh.addColor(kinect1.getColorAt(x,y));
				mesh.addVertex(kinect1.getWorldCoordinateAt(x, y));
			}
		}
	}
	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	glEnable(GL_DEPTH_TEST);
	mesh.drawVertices();
	glDisable(GL_DEPTH_TEST);
	ofPopMatrix();
}

//--------------------------------------------------------------
void testApp::exit() {
	kinect0.setCameraTiltAngle(0); // zero the tilt on exit
	kinect0.close();
	
    kinect1.setCameraTiltAngle(0); // zero the tilt on exit
	kinect1.close();
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
			
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
			
		case 'w':
			kinect0.enableDepthNearValueWhite(!kinect0.isDepthNearValueWhite());
			break;
			
		case 'o':
			kinect0.setCameraTiltAngle(angle); // go back to prev tilt
			kinect0.open();
            kinect1.setCameraTiltAngle(angle); // go back to prev tilt
			kinect1.open();
			break;
			
		case 'c':
			kinect0.setCameraTiltAngle(0); // zero the tilt
			kinect0.close();
            kinect1.setCameraTiltAngle(0); // zero the tilt
			kinect1.close();
			break;
            
        case 'b':
			backImage0.setFromPixels(kinect0.getDepthPixels(), kinect0.width, kinect0.height);
			backImage1.setFromPixels(kinect1.getDepthPixels(), kinect1.width, kinect1.height);
			break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect0.setCameraTiltAngle(angle);
            kinect1.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect0.setCameraTiltAngle(angle);
            kinect1.setCameraTiltAngle(angle);
			break;
	}
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}
