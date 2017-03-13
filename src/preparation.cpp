#include "Eigen/Dense"
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <string>

//Constants.
std::string PATH_NAME;
std::string PATH_LENGTH;
//Declaration of functions.
void shortenPathFile(std::string original, std::string path_length);
void copyPathFile(std::string original);

int main(int argc, char** argv){
	//Init ROS.
	ros::init(argc, argv, "preparation");
	ros::NodeHandle node;
	//Getting parameter.
	PATH_NAME = argv[0];
	node.getParam("/general/PATH_LENGTH", PATH_LENGTH);
	//Edit path file, iff repeat mode.
	if(PATH_LENGTH != "full") shortenPathFile(PATH_NAME, PATH_LENGTH);
	if(PATH_LENGTH == "full") copyPathFile(PATH_NAME);
	return 0;
}

void shortenPathFile(std::string original, std::string path_length_string){
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
	            <<original_filename<<std::endl;
	}
	//Getting original file length.
  	fin.seekg (-2, fin.end); 
  	int file_length = fin.tellg();
  	std::cout << "file length: " << file_length << std::endl;
  	fin.seekg (0, fin.beg);
	//Creating char array containing the original file.
	char * file = new char [file_length+1];
	fin.read(file,file_length);
	std::istringstream in_stream(file,std::ios::in);
	delete[] file;  
	fin.close ();
	//Open output file.
	std::ofstream fout;
	std::string edited_filename = original + "_edited.txt";
	fout.open(edited_filename.c_str());
	if(!fout.is_open()){
	   std::cout<<std::endl<<"PREPARATION: Error with opening of  "
	            <<edited_filename<<std::endl;
	}
	//Writing into new file until max distance is reached.
	int i = 0;
	int array_index;  
	Eigen::Vector3d position(0,0,0);
	Eigen::Vector3d last_position(0,0,0);
	Eigen::Vector4d quat;
	double velocity;
	bool stop;
	double path_distance = 0.0;
	while(position.norm()<path_length && !in_stream.eof()){
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
      path_distance += (last_position-position).norm();
      last_position = position;
	}
	fout.close();
}

void copyPathFile(std::string original){
	//Open original file.
	std::string original_filename = original + ".txt";
	std::ifstream in_file(original_filename.c_str());
	//Open edit file.
	std::string edited_filename = original + "_edited.txt";
    std::ofstream out_file(edited_filename.c_str());
    //Copy file.
    out_file << in_file.rdbuf();	
}
