
float bins;
float bearing;

void pixelPlace( float theta, unsigned int distance, unsigned int depth);
int genRand( int n );
void printascii( void );
void drawScene(unsigned int x, unsigned int y, unsigned int depth);
void initGL(int width, int height);
void bearingCallback(const std_msgs::Float32::ConstPtr& sonarBearing);
void binsCallback(const std_msgs::Float32::ConstPtr& sonarBins);

