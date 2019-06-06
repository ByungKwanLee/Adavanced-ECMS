#include <iostream>
#include <vector>

using namespace std;



class data_loader
{
public:
    static vector<float> get_1d_data(string file_name, bool print = false);
    static vector<vector<float>> get_2d_data(string file_name, bool printprint = false); 

};

