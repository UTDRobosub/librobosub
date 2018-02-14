#include <robosub/robosub.h>

using namespace robosub;

int main(int argc, char** argv) {
  DataBucket current;
  DataBucket previous;

  int i=0;
  while(true) {
    previous = current;
    current["test"] = i++;

    robosub::Time::waitMillis(500);

    DataBucket compressed = current.compress(previous);

    cout << previous.toString() << endl;
    cout << current.toString() << endl;
    cout << compressed.toString() << endl;
    cout << compressed.inflate(previous) << endl;
  }

  return 0;
}
