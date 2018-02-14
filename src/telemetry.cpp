#include "robosub/telemetry.h"

namespace robosub {

  long long DataBucket::_getUUID() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  }

  DataBucket::DataBucket() {
    _data = json({ });
    _data["data"] = json({ });
    _data["uuid"] = _getUUID();
  }

  DataBucket::DataBucket(json data) {
    _data = data;
    _data["data"] = _data;
    _data["uuid"] = _getUUID();
  }

  DataBucket::DataBucket(string string) {
    _data = json::parse(string);
    _data["data"] = _data;
    _data["uuid"] = _getUUID();
  }

  DataBucket::DataBucket(vector<uint8_t> cborFormat) {
    _data = json::from_cbor(cborFormat);
    _data["data"] = _data;
    _data["uuid"] = _getUUID();
  }

  bool DataBucket::isCompressed() {
    return _data["locked"] == true;
  }

  DataBucket::reference DataBucket::operator[] (const string key) {
    if (isCompressed()) {
      throw runtime_error("Accessors not allowed on compressed buckets");
    }
    return _data["data"][key];
  }

  DataBucket& DataBucket::operator= (const DataBucket& other) {
    _data = other._data;
    return *this;
  }

  json DataBucket::toJson() {
    return _data["data"];
  }

  string DataBucket::toString() {
    return _data["data"].dump();
  }

  string DataBucket::toPrettyString() {
    return _data["data"].dump(4); //number of spaces to indent
  }

  vector<uint8_t> DataBucket::toCbor() {
    return json::to_cbor(_data["data"]);
  }

  void DataBucket::clear() {
    _data.clear();
    _data["data"] = { };
    _data["uuid"] = _getUUID();
    _data["locked"] = false;
  }

  DataBucket DataBucket::compress(DataBucket& previousState) {
    json diffs = json({ });

    diffs["data"] = json::diff(previousState._data["data"], _data["data"]);
    diffs["uuid"] = previousState._data["uuid"];
    diffs["locked"] = true;

    DataBucket newBucket;
    newBucket._data = diffs;
    return newBucket;
  }

  bool DataBucket::isInflatable(DataBucket& previousState) {
    return (previousState._data["uuid"] == _data["uuid"]) && (isCompressed() && !previousState.isCompressed());
  }

  DataBucket DataBucket::inflate(DataBucket& previousState) {
    cout << previousState._data << endl;
    cout << _data << endl;

    if (!isInflatable(previousState))
      throw runtime_error("Cannot inflate from old or mismatched data");

    json result = json({ });

    result["data"] = previousState._data["data"].patch(_data["data"]);
    result["uuid"] = _getUUID();
    result["locked"] = true;

    DataBucket newBucket;
    newBucket._data = result;
    return newBucket;
  }

}
