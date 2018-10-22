/*
 * Util.cpp 
 */

#include "util.h"
#include<iostream>
#include<fstream>
#include<sstream>
using namespace std;

namespace exploringBB {

/**
 * Helper write function that writes a single string value to a file in the path provided
 * @param path The sysfs path of the file to be modified
 * @param filename The file to be written to in that path
 * @param value The value to be written to the file
 * @return
 */
int write(string path, string filename, string value){
   ofstream fs;
   fs.open((path + filename).c_str());
   if (!fs.is_open()){
	   perror("GPIO: write failed to open file ");
	   return -1;
   }
   fs << value;
   fs.close();
   return 0;
}
/**
 * Helper read function that reads a single string value to a file from the path provided
 * @param path The sysfs path of the file to be read
 * @param filename Filename The file to be written to in that path
 * @return
 */
string read(string path, string filename){
   ifstream fs;
   fs.open((path + filename).c_str());
   if (!fs.is_open()){
	   perror("GPIO: read failed to open file ");
    }
   string input;
   getline(fs,input);
   fs.close();
   return input;
}

/**
 * Private write method that writes a single int value to a file in the path provided
 * @param path The sysfs path of the file to be modified
 * @param filename The file to be written to in that path
 * @param value The int value to be written to the file
 * @return
 */
int write(string path, string filename, int value){
   stringstream s;
   s << value;
   return write(path,filename,s.str());
}

} /* namespace exploringBB */
