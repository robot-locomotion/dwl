#include <dwl/utils/CollectData.h>
#include <cstdlib>


int main(int argc, char **argv)
{
	std::string filename = "dwl_data.dat";
	dwl::utils::CollectData::Tags data_tags;
	data_tags.push_back("x");
	data_tags.push_back("y");
	data_tags.push_back("z");

	// Initializing the collecting data process
	dwl::utils::CollectData cd;
	cd.initCollectData(filename, data_tags);

	// Emulating the collecting data process
	std::cout << "Writing data:" << std::endl;
	for (unsigned int k = 0; k < 10; k++) {
		// Generating a random full data
		dwl::utils::CollectData::Dict data;
		for (unsigned int t = 0; t < data_tags.size(); t++) {

			// Generate a random number between 1 and 10
			double value = rand() % 10 + 1;
			data[data_tags[t]] = value;
			std::cout << value << '\t';
		}
		std::cout << std::endl;

		// Writing the new data
		cd.writeNewData(data);
	}

	// Stop collecting data process
	cd.stopCollectData();
}
