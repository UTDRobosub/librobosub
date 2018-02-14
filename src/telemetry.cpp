#include "robosub/telemetry.h"

namespace robosub {

  DataBucket::DataBucket() {
    _uuid = _uuidgen();
  }

  DataBucket::reference DataBucket::operator[] (const string key) {
    return _data[key];
  }

  json DataBucket::toJson() {
    return _data;
  }

  string DataBucket::toString() {
    return _data.dump();
  }

  string DataBucket::toPrettyString() {
    return _data.dump(4); //number of spaces to indent
  }

  DataBucket& DataBucket::compress(DataBucket previousState) {

  }

}
