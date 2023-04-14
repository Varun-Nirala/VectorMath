#ifndef __HELPER_H__
#define __HELPER_H__

#include <cassert>
#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <type_traits>

using UChar = unsigned char;
using DType = float;

#define LOG_ERROR(msg)  std::cerr << "ERROR :: " << __FUNCTION__ << "::" << __LINE__ << ":: " << msg << std::endl
#define LOG_INFO(msg)  std::cerr << "INFO :: " << __FUNCTION__ << "::" << __LINE__ << ":: " << msg << std::endl

#define PRINT_MSG(msg)  std::cerr << msg << std::endl

template<typename T>
using ENABLE_IF_INTEGER = std::enable_if_t<std::is_integral<T>::value, bool>;

template<typename T>
using ENABLE_IF_FLOAT = std::enable_if_t<std::is_floating_point<T>::value, bool>;

// Copied from from https://en.cppreference.com/w/cpp/types/numeric_limits/epsilon
template<class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
almost_equal(T x, T y, int ulp = 2)
{
    // the machine epsilon has to be scaled to the magnitude of the values used
    // and multiplied by the desired precision in ULPs (units in the last place)
    return std::fabs(x - y) <= std::numeric_limits<T>::epsilon() * std::fabs(x + y) * ulp
        // unless the result is subnormal
        || std::fabs(x - y) < std::numeric_limits<T>::min();
}

template<typename T>
inline void read1byte(const UChar* srcbuffer, int& index, T& dst)
{
    memcpy(&dst, &srcbuffer[index++], 1);
}

template<typename T>
inline void read2byte(const UChar* srcbuffer, int& index, T& dst)
{
    memcpy(&dst, &srcbuffer[index], 2);
    index += 2;
}

template<typename T>
inline void read4byte(const UChar* srcbuffer, int& index, T& dst)
{
    memcpy(&dst, &srcbuffer[index], 4);
    index += 4;
}

inline float readAsfloat(const std::string& word)
{
    return std::stof(word);
}

inline int readAsIndex(const std::string& word)
{
    return std::stoi(word);
}

inline bool splitInTokens(const std::string& line, std::vector<std::string>& vec, std::function<bool(char)> func)
{
    vec.clear();
    size_t start = 0, id = 0;
    while (id < line.size())
    {
        if (func(line[id]))
        {
            id++;
        }
        else
        {
            start = id;
            while (id < line.size() && !func(line[id]))
            {
                id++;
            }
            if (start < id)
            {
                vec.push_back(line.substr(start, id - start));
                start = id;
                id++;
            }
        }
    }

    return true;
}

static void printUsage()
{
    const std::string usage = R"(
Usage       : executable file percentage algorithm

Description : Read a targa image file and save a scaled copy of the image at the same locaton with new name.
              By default image is scaled to 50 % using the nearest neighbour algorithm.
              Excepted image extensions are tga, icb, vda and vst.

              file (required)
                 [string] this should be full path with filename and extension.
                 Example : D:\User\Files\image_name.tga

              percentage (optional)
                 [integer] this is the percentage by which we need to scale.
                 If none provided default 50 will be used.

             algorithm (optional)
                 [integer] this is the option to specify which algorithm we want to be used.
                 0 -> Nearest neighbour
                 1 -> Bilinear interpolation
                 2 -> Create two copy by using both algorithms

Options :
   -h
       Print this help.

Example :
     executable D:\User\Files\image_name.tga
     executable D:\User\Files\image_name.tga 150
     executable D:\User\Files\image_name.tga 200 0
     executable D:\User\Files\image_name.tga 50 1
     executable D:\User\Files\image_name.tga 50 2
)";

    puts(usage.c_str());
}

#endif // #ifndef __HELPER_H__