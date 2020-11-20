#include "../include/wind.h"
#include "../include/utils.h"

WindClass::WindClass() {

    windState = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    set_gust_model();

}

WindClass::WindClass(bool read_params) {

	if (read_params == 1)
		read_parameters();
	else
		WindClass();

}

void WindClass::read_parameters(){

	// read file
    std::ifstream ifs ("input/parameters.json");
    nlohmann::json json_file;

    ifs >> json_file;
    ifs.close();

    // load parameters
    windState[ew_u] = json_file["wind_parameters"]["u"];
    windState[ew_v] = json_file["wind_parameters"]["v"];
    windState[ew_w] = json_file["wind_parameters"]["w"];

    windState[ew_gust_u] = json_file["wind_parameters"]["gust_u"];
    windState[ew_gust_u] = json_file["wind_parameters"]["gust_v"];
    windState[ew_gust_u] = json_file["wind_parameters"]["gust_w"];

}

void WindClass::set_gust_model() {

    switch(GUST_MODEL) {

        case eDryden:
            gust_model = new DrydenGustModel(*this);
        case eVonKarman:
            std::exit(EXIT_FAILURE);

    }

}


void WindClass::update_gust() {

    gust_model->update_gust();

}

















