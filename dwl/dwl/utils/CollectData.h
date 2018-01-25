#ifndef DWL__UTILS__COLLECT_DATA__H
#define DWL__UTILS__COLLECT_DATA__H

#include <fstream>
#include <map>
#include <vector>
#include <limits>
#include <dwl/utils/Macros.h>


namespace dwl
{

namespace utils
{

/**
 * @class CollectData
 * @brief Generic collecting data methods
 * For starting to collect data, you will need to define the filename and the
 * different tag names of your data in the initCollectData method. Then, you
 * could write sequentially as many data as you need, with the writeNewData
 * method. Note that the data has to be specified in a dictionary. When you
 * have finished to collect data, you have to call the stopCollectData method
 */
class CollectData
{
	public:
		typedef std::map<std::string, double> Dict;
		typedef std::vector<std::string> Tags;

		/** @brief Constructor function */
		CollectData();

		/** @brief Destructor function */
		~CollectData();

		/**
		 * @brief Initializes the process of collecting data
		 * @param const std::string& File name
		 * @param const Tags& Tag list
		 */
		void initCollectData(const std::string& filename,
							const Tags& tags);

		/**
		 * @brief Write a new data to the opened file
		 * @param const Dict& Data map
		 */
		void writeNewData(const Dict& data);

		/**
		 * @brief Stops the process of collecting data
		 */
		void stopCollectData();


	private:
		/** @brief File for recording the data */
		std::ofstream datafile_;

		/** @brief Tag list */
		Tags tags_;
};

} //@namespace utils
} //@namespace dwl

#endif
