#include <string>
#include <sstream>
#include <iostream>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <algorithm>
#include <stdlib.h>

using namespace std;

vector<float> get_1d_data(string file_name)
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

    // print out what was read in
    for (size_t i=0; i<v_float.size(); ++i)
    {
        cout << v_float[i] << "|"; 
    }
    cout << "\n";

    return v_float;
}


vector<vector<float>> get_2d_data(string file_name)
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

    // print out what was read in
    for (size_t i=0; i<array.size(); ++i)
    {
        for (size_t j=0; j<array[i].size(); ++j)
        {
            cout << array[i][j] << "|"; // (separate fields by |)
        }
        cout << "\n";
    }

    return array;
}