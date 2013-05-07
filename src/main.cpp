#include "testApp.h"
#include "ofAppGlutWindow.h"

int main() {
	ofAppGlutWindow window;
	ofSetupOpenGL(&window, 1800, 768, OF_WINDOW);
	ofRunApp(new testApp());
}
