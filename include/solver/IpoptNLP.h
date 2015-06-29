#ifndef DWL_IpoptNLP_H
#define DWL_IpoptNLP_H

#include <solver/Solver.h>
#include <solver/IpoptWrapper.h>
#include <IpIpoptApplication.hpp>


namespace dwl
{

namespace solver
{

class IpoptNLP : public Solver
{
	public:
		IpoptNLP();
		~IpoptNLP();

		bool init();
		bool compute(double computation_time = std::numeric_limits<double>::max());

	private:
		solver::IpoptWrapper ipopt_;
		Ipopt::SmartPtr<Ipopt::TNLP> nlp_ptr_;
		Ipopt::SmartPtr<Ipopt::IpoptApplication> app_;
};

} //@namespace solver
} //@namespace dwl

#endif
