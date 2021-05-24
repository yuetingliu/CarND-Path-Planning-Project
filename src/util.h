#ifndef UTIL_H
#define UTIL_H

#include <map>
#include <string>

using std::map;
using std::string;

/**
 * get possible lanes to change
*/
map<string, int> get_successor_states(int lane){
  // only consider lane change here
  map <string, int> states;
  if (lane == 0){
     states["LCR"] = lane + 1;
  } else if (lane == 1){
    states["LCL"] = lane - 1;
    states["LCR"] = lane + 1;
  } else if (lane == 2) {
    states["LCL"] = lane - 1;
  }

  return states;
}
#endif  // UTIL_H
