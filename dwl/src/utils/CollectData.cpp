#include <dwl/utils/CollectData.h>


namespace dwl
{

namespace utils
{

CollectData::CollectData()
{

}


CollectData::~CollectData()
{

}


void CollectData::initCollectData(const std::string& filename,
								 const Tags& tags)
{
	if (!datafile_.is_open()) {
		// Opening a new file
		datafile_.open(filename.c_str());

		// Configuration of the data file
		datafile_.precision(4);
		datafile_.setf(std::ios::fixed, std::ios::floatfield);
		datafile_.setf(std::ios::left, std::ios::adjustfield);

		// Setting up the tags
		tags_ = tags;

		// Writing the tags name in the first row
		for (unsigned int t = 0; t < tags_.size(); t++) {
			datafile_ << tags_[t] << '\t';

			// Breaking the line in the last tag
			if (t == tags_.size() - 1)
				datafile_ << std::endl;
		}

	} else
		printf(YELLOW "Warning: the data file is already opened. Note that"
				" you could open another data file after writing the current "
				"one\n" COLOR_RESET);
}


void CollectData::writeNewData(const Dict& data)
{
	// Writing the new data according to the tag order
	for (unsigned int t = 0; t < tags_.size(); t++) {
		// Getting the data of the specific tag
		Dict::const_iterator data_it = data.find(tags_[t]);
		double data_value;
		if (data_it != data.end())
			data_value = data_it->second;
		else
			data_value = std::numeric_limits<double>::quiet_NaN();

		// Adding the actual data value
		datafile_ << data_value << '\t';

		// Breaking the line in the last tag
		if (t == tags_.size() - 1)
			datafile_ << std::endl;
	}
}


void CollectData::stopCollectData()
{
	datafile_.close();
}

} //@namespace utils
} //@namespace dwl
