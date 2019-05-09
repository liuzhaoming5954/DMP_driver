# include "dmp.h"
extern float32_t value_s;	

/**
  * @brief Compute phase system's differential equation   tau*ds=-alpha*s
  * @param[in] x current state
	* @param[out] rates of change of current state
  */
void DifferentialEquation_PhaseSystem(struct struct_dmp *DMP, float32_t *s, float32_t *sd)
{
	sd[0] = -(DMP->alpha_)*s[0]/(DMP->tau_);
}

 /**
   * Integrate the phase system one time step.
   *
   * \param[in]  dt         Duration of the time step
   * \param[in]  s          Current phase state
   * \param[out] s          Updated state of dt time later
   * \param[out] sd_updated Updated rates of change of state, dt time later.
   */
//void IntegrateStep_phase(struct struct_dmp *DMP, float32_t dt, float32_t *s, float32_t *sd_updated)
void IntegrateStep_phase(struct struct_dmp *DMP, float32_t *s, float32_t *sd_updated)
{
	// Simple Euler integration
	//DifferentialEquation_PhaseSystem(s,sd_updated);
	//s[0] = s[0] + dt*sd_updated[0];
	DifferentialEquation_PhaseSystem(DMP, s, sd_updated);
	s[0] = s[0] + DMP->dt*sd_updated[0];
}

/**
  * @brief Compute DMP system's differential equation
  * @param[in] x current state
  * @param[in] s current phase state
	* @param[out] xd rates of change of current state
  */
void DifferentialEquation_DMPSystem(struct struct_dmp *DMP, float32_t *s, float32_t *x, float32_t *xd)
{
	//xd[0] = (-DMP.spring_constant_*(x[1]-DMP.attractor_state_) - DMP.damping_coefficient_*x[0])/(DMP.mass_*DMP.tau_)+(*s)*Compute_ForceFunction(s)/DMP.tau_;
	float32_t f_value[1];
//	FunctionApproximatorLWR_predict(s, f_value);
//	xd[0] = (-DMP.spring_constant_*(x[1]-DMP.attractor_state_) - DMP.damping_coefficient_*x[0])/(DMP.mass_*DMP.tau_)+(*s)*f_value[0]/DMP.tau_;
//	xd[1] = x[0]/DMP.tau_;
	FunctionApproximatorLWR_predict(DMP, s, f_value);
	xd[0] = (-DMP->spring_constant_*(x[1]-DMP->attractor_state_) - DMP->damping_coefficient_*x[0])/(DMP->mass_*DMP->tau_)+(*s)*f_value[0]/DMP->tau_;
	xd[1] = x[0]/DMP->tau_;
	
	// LZM Debug
	value_s = (*s)*f_value[0]/DMP->tau_;
}

 /**
   * Integrate the system one time step.
   *
   * \param[in]  dt         Duration of the time step
   * \param[in]  x          Current state
	 * \param[in]  xd         Current rates of change of state
   * \param[out] x			    Updated state, dt time later.
	 * \param[out] xd         Updated rates of change of state, dt time later.
   */
//void IntegrateStep_DMP(struct struct_dmp *DMP, float32_t dt, float32_t *s, float32_t *x, float32_t *xd)
void IntegrateStep_DMP(struct struct_dmp *DMP, float32_t *s, float32_t *x, float32_t *xd)
{
	// Simple Euler integration
	DifferentialEquation_DMPSystem(DMP,s,x,xd);
	//x_updated[0] = x[0] + dt*xd_updated[0];
	//x_updated[1] = x[1] + dt*xd_updated[1];
	x[0] = x[0] + DMP->dt*xd[0];
	x[1] = x[1] + DMP->dt*xd[1];
}

 /**
   * Get line representation is "y = a(x-c) + b"
   *
   * \param[in]  s         the phase term s
	 * \param[out] lines     the sum of lines 
   */
void getLines(struct struct_dmp *DMP, float32_t *input_s, float32_t *lines)
{
	for(int i_line=0; i_line < DMP->basis_number_; i_line++)
	{
		lines[i_line] = DMP->slopes_[i_line]*((*input_s) - DMP->centers_[i_line]);
		lines[i_line] += DMP->offsets_[i_line];
	}
}

 /**
   * Get kernel_activations     
   *
   * \param[in]  s                      the phase term s
	 * \param[out] kernel_activations     the sum of lines 
   */
void kernelActivations(struct struct_dmp *DMP, float32_t *input_s, float32_t *kernel_activations)
{
	int n_basis_functions = DMP->basis_number_;
	int n_samples = 1;
	int n_dims = 1;
	
	float32_t c,w,x;
	for (int bb=0; bb<n_basis_functions; bb++)
	{
		kernel_activations[bb] = 1;
		for (int i_dim=0; i_dim<n_dims; i_dim++)
    {
      c = DMP->centers_[bb];
      for (int i_s=0; i_s<n_samples; i_s++)
      {
        x = input_s[i_s];
        w = DMP->widths_[bb];      
        kernel_activations[bb] *= exp(-0.5*pow(x-c,2)/(w*w));
      }
    }	
	}
	// Normalize the basis value
	float32_t sum_kernel_activations=0.0;
	for(int bb=0; bb<n_basis_functions; bb++)
	{
		sum_kernel_activations += kernel_activations[bb];
	}
	
	for (int i_basis=0; i_basis<n_basis_functions; i_basis++)
	{
		if (sum_kernel_activations==0.0)
			// Apparently, no basis function was active. Set all to same value
			kernel_activations[i_basis] = 1.0/n_basis_functions;
		else
			// Standard case, normalize so that they sum to 1.0
			kernel_activations[i_basis] /= sum_kernel_activations;			
	}
}

 /**
   * Integrate the system one time step.
   *
   * \param[in]  input_s    current state s
   * \param[out] output  predict value
   */
void FunctionApproximatorLWR_predict(struct struct_dmp *DMP, float32_t *input_s, float32_t *output)
{
	float32_t lines_one_prealloc_[10],activations_one_prealloc_[10],pDst_[10];
//	for (int i =0; i<10; i++)
//	{
//		activations_one_prealloc_[i] = 1.0;
//	}	
	// Only 1 sample, so real-time execution is possible. No need to allocate memory.
	getLines(DMP, input_s, lines_one_prealloc_);

	// Weight the values for each line with the normalized basis function activations  
  kernelActivations(DMP, input_s,activations_one_prealloc_);
	
	// 向量一对一相乘得到新向量，对新向量求和
	// Element-by-element multiplication of two vectors, and compute the sum of the new vector
	
	arm_mult_f32(lines_one_prealloc_,activations_one_prealloc_,pDst_,DMP->basis_number_);
	
	for(int i=0; i<DMP->basis_number_; i++)
	{
		(*output) += pDst_[i];
	}
}
