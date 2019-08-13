#pragma once

#include <fstream>
#include <string>

#include <eigen3/Eigen/Core>

#include <ros/ros.h>

#define FILE_NAME_STRING_LENGTH 200

namespace RPGQ
{
    enum PARAMETER_TYPE
    {
        PARAM_TYPE_BOOL = 0,
        PARAM_TYPE_INT,
        PARAM_TYPE_UINT,
        PARAM_TYPE_DOUBLE,
        PARAM_TYPE_DOUBLE3D,
        PARAM_TYPE_UNKNOWN = 127
    };

    union Parameter_t
    {
        bool bool_;
        int int_;
        unsigned uint_;
        double double_;
        double double3D_[3];
    };

    template <unsigned NUM_PARAMS>
    class Parameters
    {
    public:
        // constructor & destructor
        Parameters(void) {};
        Parameters(const char* name);
        Parameters(const char* name, const char* defaultName);

        // print functions
        void Print(void)
        {
            std::printf("---------------------------------------------------------------\n");
            std::printf("Parameter file: %s.txt.\n", fname_);
            std::printf("---------------------------------------------------------------\n");
            for (uint8_t i = 0; i < NUM_PARAMS; i++)
            {
                std::printf("%d:\t", i);
                
                switch (paramsType_[i])
                {
                case PARAM_TYPE_BOOL:
                    if (params_[i].bool_)
                    {
                        std::printf("true\n");
                    }
                    else
                    {
                        std::printf("false\n");
                    }
                    break;

                case PARAM_TYPE_INT:
                    std::printf("%d\n", params_[i].int_);
                    break;

                case PARAM_TYPE_UINT:
                    std::printf("%u\n", params_[i].uint_);
                    break;

                case PARAM_TYPE_DOUBLE:
                    std::printf("%f\n", params_[i].double_);
                    break;

                case PARAM_TYPE_DOUBLE3D:
                    std::printf("(%f, %f, %f)\n", params_[i].double3D_[0], params_[i].double3D_[1], params_[i].double3D_[2]);
                    break;

                default:
                    std::printf("Invalid parameter type.\n");  
                }
            }
        }

        // public get functions
        const char* GetName(void) {return fname_;};
        bool GetBool(unsigned i) const
        {
            assert(i < NUM_PARAMS);
            assert(paramsType_[i] == PARAM_TYPE_BOOL);
            return params_[i].bool_;
        };
        int GetInt(unsigned i) const
        {
            assert(i < NUM_PARAMS);
            assert(paramsType_[i] == PARAM_TYPE_INT);
            return params_[i].int_;
        };
        unsigned GetUInt(unsigned i) const
        {
            assert(i < NUM_PARAMS);
            assert(paramsType_[i] == PARAM_TYPE_UINT);
            return params_[i].uint_;
        };
        double GetDouble(unsigned i) const
        {
            assert(i < NUM_PARAMS);
            assert(paramsType_[i] == PARAM_TYPE_DOUBLE);
            return params_[i].double_;
        };
        Eigen::Vector3d GetDouble3D(unsigned i) const
        {
            assert(i < NUM_PARAMS);
            assert(paramsType_[i] == PARAM_TYPE_DOUBLE3D);
            return Eigen::Vector3d(params_[i].double3D_);
        };

    private:
        // general simulation parameters variables
        char fname_[FILE_NAME_STRING_LENGTH];
        Parameter_t params_[NUM_PARAMS];
        PARAMETER_TYPE paramsType_[NUM_PARAMS];
    };

    template <unsigned NUM_PARAMS> Parameters<NUM_PARAMS>::Parameters(const char* name)
    {
        snprintf(fname_, FILE_NAME_STRING_LENGTH, "%s", name);

        std::ifstream fileHandle; 
        fileHandle.open(std::string(getenv("RPGQ_PARAM_DIR")) + std::string("/rpgq_parameters/") + fname_ + ".txt");
        if (fileHandle.is_open())
        {
            uint8_t i = 0;

            std::string line;
            std::size_t idxStart, idxEnd;
            while (std::getline(fileHandle, line))
            {
                if (line.front() != '#')
                {
                    std::string temp;

                    // parameter number
                    idxStart = 0;
                    idxEnd = line.find(":", idxStart);
                    if (idxEnd == std::string::npos)
                    {
                        ROS_ERROR("Invalid syntax in parameter file %s.txt.\n", fname_);
                        break;
                    }
                    temp = line.substr(idxStart, idxEnd + 1 - idxStart);
                    temp.erase(std::remove(temp.begin(), temp.end(), ' '), temp.end());
                    int paramNumber = std::stoi(temp);
                    
                    // parameter type
                    idxStart = idxEnd + 1;
                    idxEnd = line.find(":", idxStart);
                    if (idxEnd == std::string::npos)
                    {
                        ROS_ERROR("Invalid syntax in parameter file %s.txt.\n", fname_);
                        break;
                    }
                    temp = line.substr(idxStart, idxEnd - idxStart);
                    temp.erase(std::remove(temp.begin(), temp.end(), ' '), temp.end());
                    PARAMETER_TYPE paramType = (PARAMETER_TYPE) std::stoi(temp);

                    // parameter value
                    idxStart = idxEnd + 1;
                    idxEnd = line.find(":", idxStart);
                    if (idxEnd == std::string::npos)
                    {
                        ROS_ERROR("Invalid syntax in parameter file %s.txt.\n", fname_);
                        break;
                    }
                    temp = line.substr(idxStart, idxEnd - idxStart);
                    temp.erase(std::remove(temp.begin(), temp.end(), ' '), temp.end());
                    
                    switch (paramType)
                    {
                        case PARAM_TYPE_BOOL:
                            paramsType_[paramNumber] = paramType;
                            params_[paramNumber].bool_ = (bool) std::stoi(temp);
                            break;

                        case PARAM_TYPE_INT:
                            paramsType_[paramNumber] = paramType;
                            params_[paramNumber].int_ = std::stoi(temp);
                            break;

                        case PARAM_TYPE_UINT:
                            paramsType_[paramNumber] = paramType;
                            params_[paramNumber].uint_ = (unsigned) std::stoi(temp);
                            break;

                        case PARAM_TYPE_DOUBLE:
                            paramsType_[paramNumber] = paramType;
                            params_[paramNumber].double_ = std::stod(temp);
                            break;

                        case PARAM_TYPE_DOUBLE3D:
                        {
                            paramsType_[paramNumber] = paramType;
                            idxStart = 0;
                            for (uint8_t j = 0; j < 2; j++)
                            {
                                idxEnd = temp.find(",", idxStart);
                                if (idxEnd == std::string::npos)
                                {
                                    ROS_ERROR("Invalid syntax in parameter file %s.txt.\n", fname_);
                                    break;
                                }
                                std::string temp_double = temp.substr(idxStart, idxEnd + 1 - idxStart);
                                temp_double.erase(std::remove(temp_double.begin(), temp_double.end(), ' '), temp_double.end());
                                params_[paramNumber].double3D_[j] = std::stod(temp_double);

                                idxStart = idxEnd + 1;
                            }
                            std::string temp_double = temp.substr(idxStart, 100);
                            temp_double.erase(std::remove(temp_double.begin(), temp_double.end(), ' '), temp_double.end());
                            params_[paramNumber].double3D_[2] = std::stod(temp_double);

                            break; 
                        }
                        default:
                            paramsType_[paramNumber] = PARAM_TYPE_UNKNOWN;
                        break;
                    }

                    // increase parameter count
                    i++;
                }
            }

            if (i != NUM_PARAMS)
            {
                ROS_WARN("Not enough parameters in parameter file %s.txt.\n", fname_);
            }

            fileHandle.close();
        }
        else
        {
            ROS_ERROR("Could not open parameter file %s.txt.\n", fname_);
        }
    };

    template <unsigned NUM_PARAMS> Parameters<NUM_PARAMS>::Parameters(const char* name, const char* defaultName)
    {
        char fname[FILE_NAME_STRING_LENGTH];
        snprintf(fname, FILE_NAME_STRING_LENGTH, "%s", name);

        // check whether parameter file exists, otherwise take default file
        std::ifstream fileHandle; 
        fileHandle.open(std::string(getenv("RPGQ_PARAM_DIR")) + std::string("/rpgq_parameters/") + fname + ".txt");
        if (fileHandle.is_open())
        {
            fileHandle.close();
            Parameters(name);
        }
        else
        {
            ROS_WARN("Using default parameter file %s.txt instead of %s.txt\n", defaultName, name);
            Parameters(defaultName);
        }
    };
}