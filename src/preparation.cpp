#include "Eigen/Dense"
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <string>

//Constants.
bool MODE;
std::string LAST_PATH_FILENAME_EDITED;
std::string LAST_PATH_FILENAME_ORIGINAL;
std::string PATH_LENGTH;
//Declaration of functions.
void shortenPathFile(std::string original, std::string edited, std::string path_length);
void copyPathFile(std::string original, std::string edited);

int main(int argc, char** argv){
	//Init ROS.
	ros::init(argc, argv, "preparation");
	ros::NodeHandle node;
	//Getting parameter.
	node.getParam("/generel/MODE_INIT", MODE);
	node.getParam("/files/LAST_PATH_FILENAME", LAST_PATH_FILENAME_EDITED);
	node.getParam("/files/LAST_PATH_ORIGINAL_FILENAME", LAST_PATH_FILENAME_ORIGINAL);
	node.getParam("/general/PATH_LENGTH", PATH_LENGTH);
	//Edit path file, iff repeat mode.
	if(MODE && PATH_LENGTH != "full") shortenPathFile(LAST_PATH_FILENAME_ORIGINAL, LAST_PATH_FILENAME_EDITED, PATH_LENGTH);
	if(MODE && PATH_LENGTH == "full") copyPathFile(LAST_PATH_FILENAME_ORIGINAL, LAST_PATH_FILENAME_EDITED);
	return 0;
}

void shortenPathFile(std::string original, std::string edited, std::string path_length_string){
	//Get path length from string.
	char buffer[20];
	for (int i=0; i<path_length_string.size(); ++i){
    	buffer[i] = path_length_string[i];
  	}
  	double path_length = atof(buffer);
	//Read original file.
	std::fstream fin;
	std::string original_filename = original + ".txt";
	fin.open(original_filename.c_str());
	if(!fin.is_open()){
	   std::cout<<std::endl<<"PREPARATION: Error with opening of  "
	            <<original<<std::endl;
	}
	//Getting original file length.
	fin.seekg(-1, std::ios_base::end);
	size_t file_length = fin.tellg();
	fin.seekg(0, std::ios_base::beg);
	//Creating char array containing the original file.
	char * file = new char [file_length+1];
	fin.read(file,file_length);
	std::istringstream in_stream(file,std::ios::in);
	delete[] file;  
	fin.close ();
	//Open output file.
	std::ofstream fout;
	std::string edited_filename = edited + ".txt";
	fout.open(edited_filename.c_str());
	if(!fout.is_open()){
	   std::cout<<std::endl<<"PREPARATION: Error with opening of  "
	            <<edited<<std::endl;
	}
	//Writing into new file until max distance is reached.
	int i = 0;
	int array_index;  
	Eigen::Vector3d position(0,0,0);
	Eigen::Vector4d quat;
	double velocity;
	bool stop;
	while(position.norm()<path_length && i<=file_length && !in_stream.eof()){
	  //Read.
	  in_stream>>array_index;
	  in_stream>>position(0);
	  in_stream>>position(1);
	  in_stream>>position(2);
	  in_stream>>quat(0);
	  in_stream>>quat(1);
	  in_stream>>quat(2);
	  in_stream>>quat(3);
	  in_stream>>velocity;
	  in_stream>>stop;
	  in_stream.ignore (300, '|');
	  //Write.
	  fout<<array_index<<" "<<
           position(0)<<" "<<position(1)<<" "<<position(2)<<" "<<
           quat(0)<<" "<<quat(1)<<" "<<quat(2)<<" "<<quat(3)<<" "<<
           velocity <<" "<<stop<<"|";
      //Indexing.
      i++;
	}
	fout.close();
}

void copyPathFile(std::string original, std::string edited){
	//Open original file.
	std::string original_filename = original + ".txt";
	std::ifstream in_file(original_filename.c_str());
	//Open edit file.
	std::string edited_filename = edited + ".txt";
    std::ofstream out_file(edited_filename.c_str());
    //Copy file.
    out_file << in_file.rdbuf();	
}
