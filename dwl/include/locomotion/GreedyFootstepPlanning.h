#ifndef DWL__LOCOMOTION__GREEDY_FOOTSTEP_PLANNING__H
#define DWL__LOCOMOTION__GREEDY_FOOTSTEP_PLANNING__H

#include <locomotion/ContactPlanning.h>


namespace dwl
{

namespace locomotion
{

/**
 * @class GreedyFootstepPlanning
 * @brief GreedyFootstepPlanning computes a footstep plan using a greedy approach. This class derive
 * from ContactPlanning
 */
class GreedyFootstepPlanning : public ContactPlanning
{
	public:
		/** @brief Constructor function */
		GreedyFootstepPlanning(bool remove_footholds,
							   double threshold_distance);

		/** @brief Destructor function */
		~GreedyFootstepPlanning();

		/**
		 * @brief Computes the contacts given a current pose of the robot
		 * @param std::vector<Contact>& Set of contacts
		 * @param std::vector<Pose> Goal pose
		 */
		bool computeContactSequence(std::vector<Contact>& contact_sequence,
									std::vector<Pose> pose_trajectory);

		/**
		 * @brief Computes the contacts given a current pose of the robot
		 * @param std::vector<Contact>& Set of contacts
		 * @param std::vector<Contact> Current contacts
		 * @param Pose Goal pose
		 */
		bool computeContacts(std::vector<Contact>& contacts,
							 std::vector<Contact> current_contacts,
							 Pose goal_pose);


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

} //@namespace locomotion
} //@namespace dwl

#endif
