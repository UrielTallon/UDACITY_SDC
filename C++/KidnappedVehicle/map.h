#ifndef MAP_H_
#define MAP_H_

#include <vector>

class Map {
public:

	struct single_landmark_s {
		int id_i;	// landmark ID
		float x_f;	// landmark x-position on the map (global coordinate system)
		float y_f;	// landmark x-position on the map (global coordinate system)
	};

	std::vector<single_landmark_s> landmark_list; // List of landmarks in the map

};

#endif // !MAP_H_
