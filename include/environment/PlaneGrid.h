#ifndef DWL_SpaceDiscretization_H
#define DWL_SpaceDiscretization_H

#include <math.h>
#include <utils/utils.h>


namespace dwl
{

namespace environment
{

/**
 * @class SpaceDiscretization
 * @brief Class for defining the discretization of environment and states
 */
class SpaceDiscretization
{
	public:
		/**
		 * @brief Constructor function
		 * @param double environment_resolution Resolution of the environment
		 */
		SpaceDiscretization(double environment_resolution);

		/**
		 * @brief Constructor function
		 * @param double environment_resolution Resolution of the environment
		 * @param double position_resolution Resolution of the position variables of the state
		 */
		SpaceDiscretization(double environment_resolution, double position_resolution);

		/**
		 * @brief Constructor function
		 * @param double environment_resolution Resolution of the environment
		 * @param double position_resolution Resolution of the position variables of the state
		 * @param double angular_resolution Resolution of the angular variables of the state
		 */
		SpaceDiscretization(double environment_resolution, double position_resolution, double angular_resolution);

		/** @brief Destructor function **/
		~SpaceDiscretization();

		/**
		 * @brief Converts a 3d coordinate into a 3d key with boundary checking.
		 * @param dwl::Key& key values that represents a 3d position
		 * @param const Eigen::Vector3d coordinate 3d cartesian coordinate of a point
		 * @return bool Returns true if the point is within the boundaries of the environment (valid), false otherwise
		 */
		bool coordToKeyChecked(Key& key, const Eigen::Vector3d coordinate) const;

		/**
		 * @brief Converts a coordinate into a key with boundary checking.
		 * @param unsigned short int& key Key value of a single axis
		 * @param const double coordinate Cartesian coordinate of a single axis
		 * @param const bool plane Indicates if the key represents a plane or a height
		 * @return bool Returns true if the point is within the boundaries of the environment (valid), false otherwise
		 */
		bool coordToKeyChecked(unsigned short int& key, const double coordinate, const bool plane) const;

		/**
		 * @brief Converts a single coordinate into a key
		 * @param unsigned short int& key Key
		 * @param const double coordinate Cartesian coordinate of a single axis
		 * @param const bool plane Indicates if the key represents a plane or a height
		 */
		void coordToKey(unsigned short int& key, const double coordinate, const bool plane) const;

		/**
		 * @brief Converts a key into a single coordinate
		 * @param double& coordinate Single coordinate
		 * @param const unsigned short int key The value of the key
		 * @param const bool plane Indicates if the key represents a plane or a height
		 */
		void keyToCoord(double& coordinate, const unsigned short int key, const bool plane) const;

		/**
		 * @brief Converts the key to vertex id
		 * @param dwl::Vertex& vertex Vertex id
		 * @param const dwl::Key key Key value
		 * @param const bool plane Indicates if the key represents a plane (2d) or a volumen (3d)
		 */
		void keyToVertex(Vertex& vertex, const Key key, const bool plane) const;

		/**
		 * @brief Converts the vertex id to key
		 * @param dwl::Key& key Key
		 * @param const dwl::Vertex vertex Vertex id
		 * @param const bool plane Indicates if the key represents a plane (2d) or a volumen (3d)
		 */
		void vertexToKey(Key& key, const Vertex vertex, const bool plane) const;

		/**
		 * @brief Converts 2d coordinate to vertex id
		 * @param dwl::Vertex& vertex Vertex id
		 * @param const Eigen::Vector2d coordinate 2d coordinate
		 */
		void coordToVertex(Vertex& vertex, const Eigen::Vector2d coordinate) const;

		/**
		 * @brief Converts 3d coordinate to vertex id
		 * @param dwl::Vertex& vertex Vertex
		 * @param const Eigen::Vector3d coordinate 3d coordinate
		 */
		void coordToVertex(Vertex& vertex, const Eigen::Vector3d coordinate) const;

		/**
		 * @brief Converts vertex id to 2d coordinate
		 * @param Eigen::Vector2d& coordinate 2d coordinate
		 * @param const dwl::Vertex vertex Vertex id
		 */
		void vertexToCoord(Eigen::Vector2d& coordinate, const Vertex vertex) const;

		/**
		 * @brief Converts vertex id to 3d coordinate
		 * @param Eigen::Vector3d& coordinate Coordinate
		 * @param const dwl::Vertex vertex Vertex id
		 */
		void vertexToCoord(Eigen::Vector3d& coordinate, const Vertex vertex) const;

		/**
		 * @brief Converts a state into a key
		 * @param unsigned short int& key Key
		 * @param double state State value
		 * @param const bool position Indicates if it's a position variable, or an angular variable
		 */
		void stateToKey(unsigned short int& key, double state, const bool position) const;

		/**
		 * @brief Converts a key into a state value
		 * @param double& state Single state
		 * @param const unsigned short int key The value of the key
		 * @param const bool position Indicates if it's a position variable, or an angular variable
		 */
		void keyToState(double& state, const unsigned short int key, const bool position) const;

		/**
		 * @brief Converts state (x,y) to a vertex
		 * @param Vertex& vertex Vertex id
		 * @param const Eigen::Vector2d state State
		 */
		void stateToVertex(Vertex& vertex, const Eigen::Vector2d state) const;

		/**
		 * @brief Converts state (x,y,yaw) to a vertex
		 * @param Vertex& vertex Vertex id
		 * @param const Eigen::Vector3d state State
		 */
		void stateToVertex(Vertex& vertex, const Eigen::Vector3d state) const;

		/**
		 * @brief Converts a vertex to a 2d state (x,y)
		 * @param Eigen::Vector2d& state 2d state
		 * @param const dwl::Vertex vertex Vertex id
		 */
		void vertexToState(Eigen::Vector2d& state, const Vertex vertex) const;

		/**
		 * @brief Converts a vertex to a 3d state (x,y,yaw)
		 * @param Eigen::Vector3d& state 3d state
		 * @param const dwl::Vertex vertex Vertex id
		 */
		void vertexToState(Eigen::Vector3d& state, const Vertex vertex) const;

		/**
		 * @brief Converts a state vertex to an environment vertex
		 * @param dwl::Vertex& environment_vertex Environment vertex (x,y)
		 * @param const dwl::Vertex state_vertex State vertex
		 * @param State state Definition of the state, i.e. XY, XY_Y, etc.
		 */
		void stateVertexToEnvironmentVertex(Vertex& environment_vertex, const Vertex state_vertex, State state) const;

		/*
		 * @brief Gets the resolution of the environment
		 * @param const bool plane Indicates if the key represents a plane or a height
		 * @return double Returns the resolution of the environment
		 */
		double getEnvironmentResolution(const bool plane);

		/*
		 * @brief Gets the resolution of the state
		 * @param bool position Indicates if it's a position or angular state
		 * @return double Returns the resolution of the state
		 */
		double getStateResolution(bool position);

		/**
		 * @brief Sets the resolution of the environment
		 * @param double resolution Resolution of the environment
		 * @param const bool plane Indicates if the key represents a plane or a height
		 */
		void setEnvironmentResolution(double resolution, const bool plane);

		/**
		 * @brief Sets the resolution of the state
		 * @param double position_resolution Resolution of the position's state
		 * @param double angular_resolution Resolution of the angular's state
		 */
		void setStateResolution(double position_resolution, double angular_resolution = 0);


	private:
		/** @brief The maximun number of discreted key */
		const unsigned short int max_key_val_;

		/** @brief The resolution of the plane of the environment */
		double plane_environment_resolution_;  ///< in meters

		/** @brief The resolution of the height of the environment */
		double height_environment_resolution_;  ///< in meters

		/** @brief The resolution of the position's variables of the state */
		double position_resolution_;  ///< in meters

		/** @brief The resolution of the angular's variables of the state */
		double angular_resolution_;  ///< in radians
};

} //@namespace environment
} //@namespace dwl


#endif
