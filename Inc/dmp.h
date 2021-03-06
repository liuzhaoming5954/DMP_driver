#include "main.h"

	struct struct_dmp{
		float32_t dt; //采样时间
		float32_t tau_; //缩放参数
		float32_t alpha_;
		//float32_t beta_;
		//float32_t c[20];
		//float32_t sigma[20];
		uint32_t  basis_number_;
		//float32_t omega[20];
		
		// Dynamical System
		float32_t initial_state_;
		float32_t attractor_state_;
		//float32_t damping_coefficient_;
		//float32_t spring_constant_;
		//float32_t mass_;
		float32_t K_;
		float32_t D_;
		
		// Function Approximators
		float32_t centers_[20]; // n_centers X n_dims
		//float32_t widths_[10];  // n_centers X n_dims
		//float32_t slopes_[10];  // n_centers X n_dims
		//float32_t offsets_[10]; // n_centers X 1
		float32_t weight_[20];
		//bool asymmetric_kernels_; // should be const
		int stamp_number;
		float32_t d,h;
	};
	
	
void DifferentialEquation_PhaseSystem(struct struct_dmp *DMP, float32_t *s, float32_t *sd);
//void IntegrateStep_phase(struct struct_dmp *DMP, float32_t dt, float32_t *s, float32_t *sd);
void IntegrateStep_phase(struct struct_dmp *DMP, float32_t *s, float32_t *sd);
void DifferentialEquation_DMPSystem(struct struct_dmp *DMP, float32_t *s, float32_t *x,float32_t *xd);
//void IntegrateStep_DMP(struct struct_dmp *DMP, float32_t dt, float32_t *s, float32_t *x, float32_t *xd);
void IntegrateStep_DMP(struct struct_dmp *DMP, float32_t *s, float32_t *x, float32_t *xd);
void getLines(struct struct_dmp *DMP, float32_t *input_s, float32_t *lines);
void FunctionApproximatorLWR_predict(struct struct_dmp *DMP, float32_t *input_s, float32_t *output);
void FunctionApproximatorNBFS_predict(struct struct_dmp *DMP, float32_t *input_s, float32_t *output);

