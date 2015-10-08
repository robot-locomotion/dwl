#include <model/Constraint.h>


namespace dwl
{

namespace model
{

Constraint::Constraint() : constraint_dimension_(0)
{
	state_buffer_.set_capacity(4);
}


Constraint::~Constraint()
{

}


void Constraint::modelFromURDFFile(std::string filename,
								   bool info)
{
	// Reading the file
	std::ifstream model_file(filename.c_str());
	if (!model_file) {
		std::cerr << "Error opening file '" << filename << "'." << std::endl;
		abort();
	}

	// Reserving memory for the contents of the file
	std::string model_xml_string;
	model_file.seekg(0, std::ios::end);
	model_xml_string.reserve(model_file.tellg());
	model_file.seekg(0, std::ios::beg);
	model_xml_string.assign((std::istreambuf_iterator<char>(model_file)),
			std::istreambuf_iterator<char>());
	model_file.close();

	modelFromURDFModel(model_xml_string, info);
}


void Constraint::modelFromURDFModel(std::string urdf_model,
									bool info)
{
	// Reseting the floating-base system information given an URDF model
	system_.resetFromURDFModel(urdf_model);

	// Initializing the kinematical and dynamical model from the URDF model
	kinematics_.modelFromURDFModel(urdf_model, info);
	dynamics_.modelFromURDFModel(urdf_model, false);

	// Initializing the information of the specific constraint
	init(urdf_model);
}


void Constraint::init(std::string urdf_model,
					  bool info)
{

}


void Constraint::setLastState(LocomotionState& last_state)
{
	state_buffer_.push_front(last_state);
}


void Constraint::resetStateBuffer()
{
	unsigned int buffer_size = state_buffer_.size();
	LocomotionState empty_state(state_buffer_[0].joint_pos.size(), state_buffer_[0].contacts.size());
	for (unsigned int i = 0; i < buffer_size; i++)
		state_buffer_.push_back();
}


unsigned int Constraint::getConstraintDimension()
{
	// Getting the constraint dimension given a defined constraint function. The constraint
	// dimension will be different to zero once the constraint dimension is defined
	if (constraint_dimension_ == 0) {
		Eigen::VectorXd bound;
		getBounds(bound, bound);
		constraint_dimension_ = bound.size();
	}

	return constraint_dimension_;
}


std::string Constraint::getName()
{
	return name_;
}

} //@namespace model
} //@namespace dwl
