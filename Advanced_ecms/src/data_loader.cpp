#include <sstream>
#include <fstream>
#include <algorithm>
#include <data_loader.hpp>

using namespace std;

vector<float> data_loader::get_1d_data(string file_name, bool print)
{
    ifstream in(file_name);

    // string object
    string line, field;

    
    vector<string> v_string;
    
    // get next line in file
    getline(in, line);
    v_string.clear();
    stringstream ss(line);

    // break line into comma delimitted fields
    while (getline(ss,field,','))  
    {
        // add each field to the 1D array
        v_string.push_back(field);
    }
    
    // change type from string to float
    vector<float> v_float(v_string.size());
    std::transform(v_string.begin(), v_string.end(), v_float.begin(), [](const std::string& val)
    {
        return std::stod(val);
    });

    if(print)
    {
        // print out what was read in
        for (register int i=0; i<v_float.size(); ++i)
        {
            cout << v_float[i] << "|"; 
        }
        cout << "\n";
    }
    

    return v_float;
}


vector<vector<float>> data_loader::get_2d_data(string file_name, bool print)
{
    ifstream in(file_name);

    // string object
    string line, field;

    vector< vector<float> > array;
    vector<string> v_string;

    // get next line in file
    while ( getline(in, line) )    
    {
        v_string.clear();
        stringstream ss(line);

        // break line into comma delimitted fields
        while (getline(ss,field,','))  
        {
            // add each field to the 1D array
            v_string.push_back(field);
        }

        // change type from string to float
        vector<float> v_float(v_string.size());
        std::transform(v_string.begin(), v_string.end(), v_float.begin(), [](const std::string& val)
        {
            return std::stod(val);
        });

        // add the 1D array to the 2D array
        array.push_back(v_float);
        v_float.clear();
    }

    if(print)
    {
        // print out what was read in
        for (register int i=0; i<array.size(); ++i)
        {
            for (register int j=0; j<array[i].size(); ++j)
            {
                cout << array[i][j] << "|"; // (separate fields by |)
            }
            cout << "\n";
        }
    }
    

    return array;
}