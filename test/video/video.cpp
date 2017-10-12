#include <opencv2/opencv.hpp>
#include <cvlib/cvlib.h>

using namespace std;

void handleSignal(int sig)
{
	//Exit
	exit(0);
}

int main(int argc, char** argv)
{
	cout << "HELLO";
	cout << func();
	return 0;
}
