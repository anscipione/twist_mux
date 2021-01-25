#ifndef TWISTMUX_TWIST_MUXUTILS__H
#define TWISTMUX_TWIST_MUXUTILS__H
#include <string>

namespace twist_mux{
  
    class ParmsHolder{

       public:
            ParmsHolder(){}
 
            std::string name;
            std::string topic;
            double      timeout;
            int         priority;
    };


} //end twist_mux namespace


#endif