#include <ICEMG.hpp>
#include <interp_tool.hpp>
#include <VehicleInfo.hpp>
// #include <omp.h>

using namespace std;


// setting constant value of Tool NumGrid
const int Tool::NumGrid = 100;

// ICEMG_update
bool Tool::ICEMG_parameter_update()
{

	// stop
	if(VehicleInfo::velocity == 0 && VehicleInfo::accel <= 0)
	{
		// Voc, Rint
		Tool::Voc(); Tool::Rint();

		// EV of P estimation
		MG::P1elec_EV = Eigen::VectorXf::Zero(MG::P1elec_EV.size()); 
		Tool::PbatD_EV(); Tool::dSOC_EV(); //after constraints

		return true;
	}


	// regen
	if(VehicleInfo::P_d() <= 0 )
	{
		// Voc, Rint
		Tool::Voc(); Tool::Rint();

		// EV of W, T, Eta1
		Tool::W1_EV(); Tool::W1_EV_constr(); //W1_EV, W1_EV_constr
		Tool::T1_EV_max(); Tool::T1_EV(); 

		Eigen::VectorXf v = Tool::T1_EV_constr(); //after constraints, T1_EV
		int i=0;
		for(register int ind = 0; ind<v.size(); ind++)
		{ 
			if(isnan(v(ind)))
			{
				i+=1;
			}
			else
			{
				break;
			}
		}
		if(i==v.size()) { VehicleInfo::accel += 0.01; return false;}

		Tool::Eta1_EV();

		// EV of P estimation
		Tool::P1elec_EV(); Tool::PbatD_EV(); Tool::dSOC_EV(); //after constraints

		return true;
	}


	// Voc, Rint
	Tool::Voc(); Tool::Rint();

	// EV of W, T, Eta1
	Tool::W1_EV(); Tool::W1_EV_constr(); //W1_EV, W1_EV_constr
	Tool::T1_EV_max(); Tool::T1_EV();  //after constraints, T1_EV
	
	int i_=0;
	Eigen::VectorXf v_ = Tool::T1_EV_constr();
	for(register int ind=0; ind<v_.size();ind++)
	{
		if(isnan(v_(ind)))
		{
			i_ += 1;
		}
		else
		{
			break;
		}
	}

	// HEV of Wi, Ti
	Tool::Wi_HEV(); Tool::Wi_HEV_rep();
	Tool::Ti_max(); Tool::Ti_HEV(); Tool::FC_HEV();
	
	// HEV of W1 T1
	Tool::W1_HEV(); //after constraints, T1_EV
	
	int i__=0;
	Eigen::MatrixXf v__ = Tool::T1_HEV();
	for(register int ind=0; ind<v__.rows();ind++)
	{

		for(register int ind2=0; ind2 < v__.cols() ; ind2++)
		{
			if(isnan(v__(ind, ind2)))
			{
				i__ += 1;
			}
			else
			{
				break;
			}
		}	
	}

	if(i_ == v_.size() && i__ == v__.size()) {VehicleInfo::accel-=0.01; return false;}



	// EV of P estimation
	Tool::Eta1_HEV(); Tool::P1elec_EV(); Tool::PbatD_EV(); Tool::dSOC_EV(); //after constraints
	
	// HEV of P estimation
	Tool::Eta1_EV(); Tool::P1elec_HEV(); Tool::PbatD_HEV(); Tool::dSOC_HEV();

	return true;

}


// fundamental tool
vector<float> Tool::rpm2rs(vector<float> rpm)
{
	std::for_each(rpm.begin(), rpm.end(), [](float& x) {x*=M_PI/30;});
	return rpm;
}

float Tool::Voc(bool update)
{
	float v = MG::Bat_NumCell * interp_Tool::interpolate_1d(MG::Bat_indexSoc, MG::Bat_indexVoc, MG::SOC, false);
	if (update) MG::Voc=v;
	return v;
}

float Tool::Rint(bool update)
{
	float v = MG::Bat_NumCell * interp_Tool::interpolate_1d(MG::Bat_indexSoc, MG::Bat_indexRint,  MG::SOC, false);
	if (update) MG::Rint=v;
	return v;
}

// EV of W1, T1, Eta1 estimation
Eigen::VectorXf Tool::W1_EV(bool update) 
{
	Eigen::VectorXf v;
	v = VehicleInfo::velocity/VehicleInfo::wheel_radius*ICE::FD_Ratio*ICE::TMR;
	if(update) MG::W1_EV = v;
	return v;		
 } // nothing makes effect

 Eigen::VectorXf Tool::W1_EV_constr(bool update)
 {
	Eigen::VectorXf v_ = MG::W1_EV;

	#pragma omp parallel for
	for(register int ind=0; ind<v_.size(); ind++)
	{
		if( v_(ind) > MG::W1_EV_max || v_(ind) < -MG::W1_EV_max) v_(ind) = NAN;
	}
	
	if(update) MG::W1_EV_constr = v_;
	return v_;
 }


Eigen::VectorXf Tool::T1_EV(bool update) {

	Eigen::VectorXf v;
	v = (VehicleInfo::mass*VehicleInfo::accel + VehicleInfo::Frl())  
	* ( VehicleInfo::wheel_radius / (ICE::FD_Ratio*ICE::FD_EFF*ICE::TM_Eff) ) * ICE::TMR.cwiseInverse();

	if(update) MG::T1_EV = v;
	return v;
} //nothing makes effect

Eigen::VectorXf Tool::T1_EV_constr(bool update) {

	Eigen::VectorXf v;
	v = (VehicleInfo::mass*VehicleInfo::accel + VehicleInfo::Frl())  
	* ( VehicleInfo::wheel_radius / (ICE::FD_Ratio*ICE::FD_EFF*ICE::TM_Eff) ) * ICE::TMR.cwiseInverse();

	for(register int ind=0; ind<v.size(); ind++)
	{
		if (isnan(MG::T1_EV_max(ind))) {v(ind) = NAN; continue;}
		if( v(ind) > MG::T1_EV_max(ind) || v(ind) < -MG::T1_EV_max(ind)) v(ind) = NAN;
	}

	if(update) MG::T1_EV_constr = v;
	return v;
} //nothing makes effect


Eigen::VectorXf Tool::Eta1_EV(bool update)
{
	Eigen::VectorXf v(MG::W1_EV.size());

	#pragma omp parallel for
	for(register int ind =0; ind < v.size(); ind++)
	{
		v(ind) = interp_Tool::interpolate_2d(MG::MG_mapRPM, MG::MG_mapTrq, MG::MG_mapData, 
						abs(MG::W1_EV_constr(ind)), abs(MG::T1_EV_constr(ind)));
	}

	if(update) MG::Eta1_EV = v;
	return v;
} // W1_EV, T1_EV makes effect


// EV of P estimation
Eigen::VectorXf Tool::P1elec_EV(bool update)
{

	Eigen::VectorXf W1T1 = MG::W1_EV_constr.cwiseProduct(MG::T1_EV_constr);
	Eigen::VectorXf v(W1T1.size());	

	#pragma omp parallel for
	for(register int ind =0; ind < v.size(); ind++)
	{
		int sign = (W1T1(ind) >=0) ? 1 : -1;
		v(ind) = W1T1(ind) * pow (MG::Eta1_EV(ind), -sign);
	}

	if(update) MG::P1elec_EV = v;
	return v;
} // W1_EV, T1_EV, Eta1_EV makes effect

Eigen::VectorXf Tool::PbatD_EV(bool update)
{
	Eigen::VectorXf v;
	v = MG::P1elec_EV + MG::P_aux * Eigen::VectorXf::Ones(MG::P1elec_EV.size());

	#pragma omp parallel for
	for(register int ind=0; ind<v.size(); ind++)
	{
		if( v(ind) > MG::P_bat || v(ind) < -MG::P_bat) v(ind) = NAN;
	}

	if(update) MG::PbatD_EV = v;
	return v;

} // P1elec_EV makes effect

Eigen::VectorXf Tool::dSOC_EV(bool update)
{
	Eigen::VectorXf v(MG::PbatD_EV.size());

	#pragma omp parallel for
	for(register int ind = 0; ind<v.size(); ind++)
	{
		v(ind) =  (-MG::Voc + sqrt(pow(MG::Voc, 2)-4 * MG::PbatD_EV(ind) * MG::Rint)) / (2 * MG::Rint * MG::Bat_Quantity * 3600);
	}

	if(update) MG::dSOC_EV = v;

	return v;
} // PbatD_EV makes effect


// HEV of Wi, Ti
Eigen::VectorXf Tool::Wi_HEV(bool update)
{
	Eigen::VectorXf v = MG::W1_EV;

	#pragma omp parallel for
	for(register int ind=0; ind<v.size(); ind++)
	{
		if( v(ind) > ICE::Wi_HEV_max || v(ind) < ICE::Wi_HEV_min) v(ind) = NAN;
	}

	if(update) ICE::Wi_HEV = v;
	return v;
} // W1_EV makes effect

Eigen::MatrixXf Tool::Wi_HEV_rep(bool update)
{
	Eigen::MatrixXf v;
	v = ICE::Wi_HEV.replicate<1,Tool::NumGrid >();
	if(update) ICE::Wi_HEV_rep = v;
	return v;
} // W1_HEV makes effect

Eigen::VectorXf Tool::Ti_max(bool update)
{
	Eigen::VectorXf v(ICE::Wi_HEV.size());

	#pragma omp parallel for
	for(register int ind = 0; ind<v.size();ind++)
	{
		v(ind) = interp_Tool::interpolate_1d(ICE::En_maxRPM,ICE::En_maxTrq, ICE::Wi_HEV(ind), false);
	}

	if(update) ICE::Ti_max = v;
	return v;
} //Wi_HEV makes effect

Eigen::MatrixXf Tool::Ti_HEV(bool update)
{

	Eigen::MatrixXf v(ICE::TMR.size(), Tool::NumGrid);

	#pragma omp parallel for
	for(register int gear=0; gear<ICE::TMR.size() ;gear++)
	{	
		v.row(gear) = Eigen::VectorXf::LinSpaced(Tool::NumGrid, 0, ICE::Ti_max(gear));
	}


	#pragma omp parallel for schedule(dynamic,1) collapse(2)
	for(register int ind = 0; ind<v.rows(); ind ++)
	{
		float Ti_HEV_max = interp_Tool::interpolate_1d(ICE::En_maxRPM, ICE::En_maxTrq, ICE::Wi_HEV(ind), false);
		if (isnan(Ti_HEV_max)) v.row(ind) = NAN*Eigen::VectorXf::Ones(v.cols());
		for(register int ind2=0; ind2<v.cols(); ind2++)
		{
			if ( v(ind, ind2) > Ti_HEV_max ) v(ind, ind2) = NAN;
		}
	}


	if(update) ICE::Ti_HEV = v;
	return v;
} // Ti_max makes effect

Eigen::MatrixXf Tool::FC_HEV(bool update)
{

	Eigen::MatrixXf v(ICE::Wi_HEV_rep.rows(), ICE::Wi_HEV_rep.cols());

	#pragma omp parallel for schedule(dynamic,1) collapse(2)
	for (register int ind =0; ind < v.rows(); ind++)
	{
		for(register int ind2=0; ind2 <v.cols(); ind2++){
			v(ind, ind2) = interp_Tool::interpolate_2d(ICE::En_mapRPM, ICE::En_mapTrq, ICE::En_FC, 
				ICE::Wi_HEV_rep(ind, ind2), ICE::Ti_HEV(ind, ind2));
		}
	}

	if(update) ICE::FC_HEV = v;
	return v;
} //Wi_HEV_rep, Ti_HEV makes effect



// HEV of W1 T1 Eta1
Eigen::MatrixXf Tool::W1_HEV(bool update) 
{
	Eigen::MatrixXf v;
	v = ICE::Wi_HEV_rep;
	if(update) MG::W1_HEV = v;
	return v;
} // Wi_HEV_rep makes effect

Eigen::MatrixXf Tool::T1_HEV(bool update) 
{
	Eigen::MatrixXf v;
	v = MG::T1_EV.replicate<1, Tool::NumGrid>()-ICE::Ti_HEV;

	#pragma omp parallel for schedule(dynamic,1) collapse(2)
	for(register int ind = 0; ind<v.rows(); ind ++)
	{
		float T1_HEV_max = interp_Tool::interpolate_1d(MG::MG_maxRPM, MG::MG_maxTrq, abs(ICE::Wi_HEV(ind)), false);
		if (isnan(T1_HEV_max)) v.row(ind) = NAN*Eigen::VectorXf::Ones(v.cols());
		for(register int ind2=0; ind2<v.cols(); ind2++)
		{
			if ( v(ind, ind2) > T1_HEV_max || v(ind,ind2) < -T1_HEV_max ) v(ind, ind2) = NAN;
		}
	}

	if(update) MG::T1_HEV = v;
	return v;
} // T1_EV, Ti_HEV makes effect

Eigen::MatrixXf Tool::Eta1_HEV(bool update)
{

	Eigen::MatrixXf v(MG::W1_HEV.rows(), MG::W1_HEV.cols());

	#pragma omp parallel for schedule(dynamic,1) collapse(2)
	for (register int ind =0; ind < v.rows(); ind++)
	{
		for(register int ind2=0; ind2 <v.cols(); ind2++){
			v(ind, ind2) = interp_Tool::interpolate_2d(MG::MG_mapRPM, MG::MG_mapTrq, MG::MG_mapData, 
				abs(MG::W1_HEV(ind, ind2)), abs(MG::T1_HEV(ind, ind2)));
		}
	}
	if(update) MG::Eta1_HEV = v;
	return v;
} //W1_HEV, T1_HEV makes effect



// HEV of P estimation
Eigen::MatrixXf Tool::P1elec_HEV(bool update)
{

	Eigen::MatrixXf W1T1 = MG::W1_HEV.cwiseProduct(MG::T1_HEV);
	Eigen::MatrixXf v(W1T1.rows(), W1T1.cols());

	#pragma omp parallel for schedule(dynamic,1) collapse(2)
	for (register int ind =0; ind < v.rows(); ind++)
	{
		for(register int ind2=0; ind2<v.cols();ind2++){

			int sign = (W1T1(ind, ind2) >=0) ? 1 : -1;
			v(ind, ind2) = W1T1(ind, ind2) * pow (MG::Eta1_HEV(ind, ind2), -sign);
		}

	}
	if(update) MG::P1elec_HEV = v;
	return v;
} //W1_HEV, T1_HEV, Eta1_HEV makes effect


Eigen::MatrixXf Tool::PbatD_HEV(bool update)
{
	Eigen::MatrixXf v;
	v = MG::P1elec_HEV + MG::P_aux * Eigen::MatrixXf::Ones(MG::P1elec_HEV.rows(), MG::P1elec_HEV.cols());

	#pragma omp parallel for schedule(dynamic,1) collapse(2)
	for(register int ind=0; ind<v.rows(); ind++)
	{
		for(register int ind2=0; ind2 <v.cols(); ind2++)
		{
			if( v(ind, ind2) > MG::P_bat || v(ind, ind2) < -MG::P_bat) v(ind, ind2) = NAN;	
		}
	}

	if(update) MG::PbatD_HEV = v;
	return v;
} // PbatD_HEV makes effect


Eigen::MatrixXf Tool::dSOC_HEV(bool update)
{
	Eigen::MatrixXf v(MG::PbatD_HEV.rows(), MG::PbatD_HEV.cols());

	#pragma omp parallel for schedule(dynamic,1) collapse(2)
	for(register int ind = 0; ind<v.rows(); ind++)
	{
		for(register int ind2=0;ind2<v.cols(); ind2++)
		{
			v(ind, ind2) =  (-MG::Voc + sqrt(pow(MG::Voc, 2)-4 * MG::PbatD_HEV(ind, ind2) * MG::Rint)) / (2 * MG::Rint * MG::Bat_Quantity * 3600);
		}
		
	}
	
	if(update) MG::dSOC_HEV = v;
	return v;
} // PbatD_HEV makes effect