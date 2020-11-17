#include "../include/windy.h"
#include "../include/utils.h"

WindyClass::WindyClass() {

    windState = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


}

WindyClass::WindyClass(bool read_params) {

	if (read_params == 1)
		read_parameters();
	else
		windState = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

}

void WindyClass::read_parameters(){

	// read file
    std::ifstream ifs ("input/parameters.json");
    nlohmann::json json_file;


    windState[e_u] = json_file["wind_parameters"]["u"]
    windState[e_v] = json_file["wind_parameters"]["v"]
    windState[e_w] = json_file["wind_parameters"]["w"]

    windState[e_gust_u] = json_file["wind_parameters"]["gust_u"]
    windState[e_gust_u] = json_file["wind_parameters"]["gust_v"]
    windState[e_gust_u] = json_file["wind_parameters"]["gust_w"]





}