#ifndef DWL_GreedyFootstepPlanning_H
#define DWL_GreedyFootstepPlanning_H

#include <planning/ContactPlanning.h>


namespace dwl
{

namespace planning
{

/**
 * @class GreedyFootstepPlanning
 * @brief Class for computing footstep plan using a greedy approach
 */
class GreedyFootstepPlanning : public ContactPlanning
{
	public:
		/** @brief Constructor function */
		GreedyFootstepPlanning(bool remove_footholds, double threshold_distance);

		/** @brief Destructor function */
		~GreedyFootstepPlanning();

		/**
		 * @bief Computes the contacts given a current pose of the robot
		 * @param std::vector<Contact>& contact_sequence Set of contacts
		 * @param std::vector<dwl::Pose> pose_trajectory Goal pose
		 */
		bool computeContactSequence(std::vector<Contact>& contact_sequence, std::vector<Pose> pose_trajectory);

		/**
		 * @bief Computes the contacts given a current pose of the robot
		 * @param std::vector<Contact>& contacts Set of contacts
		 * @param std::vector<Contact> current_contacts Current contacts
		 * @param dwl::Pose goal_pose Goal pose
		 */
		bool computeContacts(std::vector<Contact>& contacts, std::vector<Contact> current_contacts, Pose goal_pose);


	private:
		/** @brief Leg offset */
		double leg_offset_;

		/** @brief The last executed past leg */
		int last_past_leg_;

		/** @brief Remove foothold tag */
		bool remove_footholds_;

		/** @brief Threshold distance for removing footholds */
		double threshold_distance_;
};

} //@namespace planning
} //@namespace dwl

#endif
