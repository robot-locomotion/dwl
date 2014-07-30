#ifndef DWL_EnvironmentInformation_H
#define DWL_EnvironmentInformation_H

#include <environment/PlaneGrid.h>
#include <utils/utils.h>


namespace dwl
{

namespace environment
{

/**
 * @class EnvironmentInformation
 * @brief Class for defining the environment information
 */
class EnvironmentInformation
{
	public:
		/** @brief Constructor function */
		EnvironmentInformation();

		/** @brief Destructor function */
		~EnvironmentInformation();

		/**
		 * @brief Sets the terrain cost map from the reward map information
		 * @param std::vector<dwl::Cell> reward_map Reward map
		 */
		void setEnvironmentInformation(std::vector<Cell> reward_map);

		/**
		 * @brief Sets the environment resolution of the plane or height
		 * @param double resolution Resolution value
		 * @param bool plane Indicates if the key represents a plane or a height
		 */
		void setEnvironmentResolution(double resolution, bool plane);

		/**
		 * @brief Sets the state resolution of the plane or height
		 * @param double position_resolution Position resolution value
		 * @param double angular_resolution Angular resolution value
		 */
		void setStateResolution(double position_resolution, double angular_resolution);

		/**
		 * @brief Gets the terrain cost-map (using vertex id)
		 * @param dwl::CostMap& costmap Cost map of the terrain
		 */
		void getTerrainCostMap(CostMap& costmap);

		/**
		 * @brief Gets the terrain height-map (using vertex id)
		 * @param dwl::HeightMap& heightmap Height map of the terrain
		 */
		void getTerrainHeightMap(HeightMap& heightmap);

		/**
		 * @brief Gets the average cost of the terrain
		 * @return double Returns the average cost of the terrain
		 */
		double getAverageCostOfTerrain();

		/**
		 * @brief Gets the defined grid model according the resolution of the environment
		 * @return dwl::environment::PlaneGrid& Returns the reference of the environment object
		 */
		const SpaceDiscretization& getSpaceModel() const;

		/**
		 * @brief Indicates if it was defined terrain information
		 * @return Returns true if it was defined terrain information, otherwise false
		 */
		bool isTerrainInformation();


	private:
		/** @brief Object of the PlaneGrid class for defining the grid routines */
		environment::SpaceDiscretization space_discretization_;

		/** @brief Gathers the cost values that are mapped using the vertex id */
		CostMap terrain_cost_map_;

		/** @brief Gathers the height values that are mapperd using the vertex id */
		HeightMap terrain_height_map_;

		/** @brief Average cost which is used for unknown areas */
		double average_cost_;

		/** @brief Indicates if it was defined terrain information */
		bool terrain_information_;
};

} //@namespace environment
} //@namespace dwl


#endif
