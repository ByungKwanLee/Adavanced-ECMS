#include <performance.hpp>
#include <VehicleInfo.hpp>
#include <ICEMG.hpp>
#include <data_loader.hpp>


using namespace std;

std::vector<float> ECMS_performance::velocity_cycle = data_loader::get_1d_data("src/advanced_ecms/data/velocity_cycle");
std::vector<float> ECMS_performance::accel_cycle    = data_loader::get_1d_data("src/advanced_ecms/data/accel_cycle");

ECMS_performance::ECMS_performance(float SOC)
{

    ECMS_performance::SOC_init = SOC;
}

void ECMS_performance::do_performance(Optimizer & obj_optimizer, string method, float time)
{

	VehicleInfo::time_rt = time;
	MG::SOC = ECMS_performance::SOC_init;

	#pragma omp pararrel for
	for (register int ind=0; ind < accel_cycle.size(); ind++)
	{	
		VehicleInfo::accel = ECMS_performance::accel_cycle[ind];
		VehicleInfo::velocity = ECMS_performance::velocity_cycle[ind];

		int num = 0;
		bool update = false;
		while(!update)
		{
			update = Tool::ICEMG_parameter_update();
			num += 1;
		}

        if(num>=2) cout << "NUM!!" << endl;

        // current soc
        ECMS_performance::SOC_per.push_back(MG::SOC); 

		obj_optimizer.optimizer(method);
        obj_optimizer.En_FC_rt(false);

        // optimal mode
        ECMS_performance::mode_per.push_back(int((std::get<0>(obj_optimizer.optimal_inform) == "HEV" ? 2 : 1) )
      + int((std::get<0>(obj_optimizer.optimal_inform) == "EV_Stop" ?  -1 : 0))
      + int((std::get<0>(obj_optimizer.optimal_inform) == "EV_Regen" ?  -2 : 0)));

        // optimal Ti
        ECMS_performance::Ti.push_back( std::get<0>(obj_optimizer.optimal_inform) == "HEV" ?
           ICE::Ti_HEV( std::get<1>(obj_optimizer.optimal_inform)-1, std::get<2>(obj_optimizer.optimal_inform) ): 0);

        // optimal Wi
        ECMS_performance::Wi.push_back(std::get<0>(obj_optimizer.optimal_inform) == "HEV" ?
           ICE::Wi_HEV( std::get<1>(obj_optimizer.optimal_inform)-1) : 0);

        if(std::get<0>(obj_optimizer.optimal_inform)=="EV_Stop")

        {
            ECMS_performance::T1.push_back(0);
            ECMS_performance::W1.push_back(0);
        }
        else
        {
            // optimal T1
            ECMS_performance::T1.push_back(std::get<0>(obj_optimizer.optimal_inform) == "HEV" ?
                MG::T1_HEV(std::get<1>(obj_optimizer.optimal_inform)-1, std::get<2>(obj_optimizer.optimal_inform))
                : MG::T1_EV_constr(std::get<1>(obj_optimizer.optimal_inform)-1));

            // optimal W1
            ECMS_performance::W1.push_back(std::get<0>(obj_optimizer.optimal_inform) == "HEV" ?
                ICE::Wi_HEV(std::get<1>(obj_optimizer.optimal_inform)-1)
                : MG::W1_EV_constr(std::get<1>(obj_optimizer.optimal_inform)-1));
        }

        
        // optimal gear
        ECMS_performance::gear_per.push_back(std::get<0>(obj_optimizer.optimal_inform)=="EV_Stop" ?
            0 : std::get<1>(obj_optimizer.optimal_inform));
        // optimal dsoc
        ECMS_performance::dSOC_per.push_back(obj_optimizer.correction); 
        // optimal mass
        ECMS_performance::mass_per.push_back(obj_optimizer.En_FC_sum);
        // optimal lambda
        ECMS_performance::lambda_per.push_back(obj_optimizer.lambda); 
        // optimal mu
        ECMS_performance::mu_per.push_back(obj_optimizer.mu); 
        // optimal raw
        ECMS_performance::raw_per.push_back(obj_optimizer.raw);
	}


	// assert( abs(MG::SOC-soc_t0) <= pow(10, -1) && "Caution : SOC constraints is not zero!");

    // print
    obj_optimizer.En_FC_rt(true);
	cout <<"dSOC (soc tf - soc t0): " << SOC_per[accel_cycle.size()-1]-ECMS_performance::SOC_init << endl;
}


void ECMS_performance::csvwrite(string method)
{
    ofstream myFile;
    myFile.open("src/advanced_ecms/output/dSOC_per.csv");

    for(register int ind = 0; ind < ECMS_performance::dSOC_per.size(); ind ++)
    {
        myFile << ind * VehicleInfo::time_rt <<
        ","<<ECMS_performance::dSOC_per[ind]<< endl;
    }
    
    myFile.close();
    myFile.open("src/advanced_ecms/output/SOC_per.csv");

    for(register int ind = 0; ind < ECMS_performance::SOC_per.size(); ind ++)
    {
        myFile << ind * VehicleInfo::time_rt <<
        ","<<ECMS_performance::SOC_per[ind]<< endl;
    }

    myFile.close();
    myFile.open("src/advanced_ecms/output/gear_per.csv");

    for(register int ind = 0; ind < ECMS_performance::gear_per.size(); ind ++)
    {
        myFile << ind * VehicleInfo::time_rt <<
        ","<<ECMS_performance::gear_per[ind]<< endl;
    }

    myFile.close();
    myFile.open("src/advanced_ecms/output/mode_per.csv");

    for(register int ind = 0; ind < ECMS_performance::mode_per.size(); ind ++)
    {
        myFile << ind * VehicleInfo::time_rt <<
        ","<<ECMS_performance::mode_per[ind]<< endl;
    }

    myFile.close();
    myFile.open("src/advanced_ecms/output/mass_per.csv");

    for(register int ind = 0; ind < ECMS_performance::mass_per.size(); ind ++)
    {
        myFile << ind * VehicleInfo::time_rt <<
        ","<<ECMS_performance::mass_per[ind]<< endl;
    }
    myFile.close();
    myFile.open("src/advanced_ecms/output/Ti_per.csv");

    for(register int ind = 0; ind < ECMS_performance::mass_per.size(); ind ++)
    {
        myFile << ind * VehicleInfo::time_rt <<
        ","<<ECMS_performance::Ti[ind]<< endl;
    }
    myFile.close();
    myFile.open("src/advanced_ecms/output/Wi_per.csv");

    for(register int ind = 0; ind < ECMS_performance::mass_per.size(); ind ++)
    {
        myFile << ind * VehicleInfo::time_rt <<
        ","<<ECMS_performance::Wi[ind]<< endl;
    }
    myFile.close();
    myFile.open("src/advanced_ecms/output/T1_per.csv");

    for(register int ind = 0; ind < ECMS_performance::mass_per.size(); ind ++)
    {
        myFile << ind * VehicleInfo::time_rt <<
        ","<<ECMS_performance::T1[ind]<< endl;
    }
    myFile.close();
    myFile.open("src/advanced_ecms/output/W1_per.csv");

    for(register int ind = 0; ind < ECMS_performance::mass_per.size(); ind ++)
    {
        myFile << ind * VehicleInfo::time_rt <<
        ","<<ECMS_performance::W1[ind]<< endl;
    }
    myFile.close();


    if(method == "L")
    {
        myFile.open("src/advanced_ecms/output/lambda_per.csv");

        for(register int ind = 0; ind < ECMS_performance::lambda_per.size(); ind ++)
        {
            myFile << ind * VehicleInfo::time_rt <<
            ","<<ECMS_performance::lambda_per[ind]<< endl;
        }

        myFile.close();
    }
    else
    {
        myFile.open("src/advanced_ecms/output/mu_per.csv");

        for(register int ind = 0; ind < ECMS_performance::mu_per.size(); ind ++)
        {
            myFile << ind * VehicleInfo::time_rt <<
            ","<<ECMS_performance::mu_per[ind]<< endl;
        }

        myFile.close();

        myFile.open("src/advanced_ecms/output/raw_per.csv");

        for(register int ind = 0; ind < ECMS_performance::raw_per.size(); ind ++)
        {
            myFile << ind * VehicleInfo::time_rt <<
            ","<<ECMS_performance::raw_per[ind]<< endl;
        }

        myFile.close();
    }
}