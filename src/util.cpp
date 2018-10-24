#include "robosub/util.h"

namespace robosub {

	void Util::pause()
	{
		std::cin.ignore();
	}

	Size Util::getDesktopResolution()
	{
#ifdef WINDOWS
		return cv::Size(GetSystemMetrics(SM_CXSCREEN), GetSystemMetrics(SM_CYSCREEN));
#else
        Display* d = XOpenDisplay(NULL);
        Screen* s = DefaultScreenOfDisplay(d);
        cv::Size z = cv::Size(s->width, s->height);
        //delete s;
        //XCloseDisplay(d);
        return z;
#endif
	}

	bool Util::directoryExists(string path) {
		struct stat buffer;
		if (stat(path.c_str(), &buffer) == 0) {
			if (buffer.st_mode & S_IFDIR)
			{
				//it's a directory
				return true;
			}
		}
		return false;
	}

	bool Util::fileExists(string path) {
		struct stat buffer;
		if (stat(path.c_str(), &buffer) == 0) {
			if (buffer.st_mode & S_IFREG)
			{
				//it's a file
				return true;
			}
		}
		return false;
	}

	vector<string> Util::splitString(string s, char by) {
		std::stringstream test(s);
		std::string segment;
		std::vector<std::string> seglist;

		while (std::getline(test, segment, by))
		{
			seglist.push_back(segment);
		}

		return seglist;
	}

	vector<Util::Match> Util::regex(string pattern, string test) {
		vector<Match> matches;

		std::regex expr(pattern);
		for (auto it = std::sregex_iterator(test.begin(), test.end(), expr);
			it != std::sregex_iterator();
			++it)
		{
			vector<string> groups;

			for (int i = 1; i < it->size(); i++) {
				groups.push_back(it->str(i));
			}

			matches.push_back(Match((string)it->str(0), groups));
		}

		return matches;
	}
	
	
	std::string Util::execCLI(std::string cmd) {
		std::array<char, 128> buffer;
		std::string result;
		std::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);
		if (!pipe) throw std::runtime_error("popen() failed!");
		while (!feof(pipe.get())) {
			if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
				result += buffer.data();
		}
		return result;
	}
	
	//Moving average class
	MovingAverage::MovingAverage(int len){
		queueLen = len;
		queueData = new int[queueLen];
		clearData();
	}
	
	void MovingAverage::insertData(int newVal){
		queuePos = (queuePos+1)%queueLen;
		
		int lastVal = queueData[queuePos];
		curSum-=lastVal;
		
		queueData[queuePos] = newVal;
		curSum+=newVal;
	}
	
	void MovingAverage::clearData(){
		queuePos = 0;
		for(int i=0; i<queueLen; i++){
			queueData[i] = 0;
		}
		curSum = 0;
	}
	
	float MovingAverage::getAverage(){
		return ((float)curSum)/((float)queueLen);
	}
}
