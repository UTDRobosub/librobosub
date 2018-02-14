#include <robosub/robosub.h>

using namespace robosub;

int main(int argc, char** argv) {
  DataBucket bucket;
  bucket["a"] = "wonderful";
  bucket["b"] = 5;
  cout << bucket.toString() << " " << bucket["a"] << endl;
  return 0;
}
