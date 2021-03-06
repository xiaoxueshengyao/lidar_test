/****
 * Description: File Manager for trajectory
 * Author:Capta1nY(Copy from renqian)
 * Data:0604
 * ***/


#ifndef FILE_MANAGER_HPP_
#define FILE_MANAGER_HPP_

#include <iostream>
#include <string>
#include <fstream>


namespace lidar_test{

class FileManager
{
    public:
        static bool CreateFile(std::ofstream& ofs, std::string file_path);
        static bool CreateDirectory(std::string directory_path);
        static bool CreateDirectory(std::string directory_path, std::string use_for);
        static bool InitDirectory(std::string directory_path, std::string use_for);



};

}


#endif