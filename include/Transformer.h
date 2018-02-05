#include<Eigen/Dense>

namespace esrocos {
  namespace transformer{

    template<int numberOfFrames = 20, int stringSize = 256>
    class AcyclicTransformer{

    private:

      static const int frames_ = numberOfFrames;
      static const int ssize_ = stringSize;

      char frames[numberOfFrames*stringSize];

      Eigen::Matrix<double, numberOfFrames-1,4> transforms;

    public:

      int frames() const {return frames_;}
      int ssize() const {return ssize_;}

      std::string frameNameAt(int i){
        char tmp[stringSize] = frames + sizeof(char)*stringSize*i;
        return std::string(tmp);
      }

      // plus one for the null terminator
      bool setFrameNameAt(std::string s, int i){
        // if the string is too long, do nothing and return false
        if (s.length()+1 > stringSize) return false;
        // does that work?
        std::strcpy(frames+sizeof(char)*i*stringSize,s.c_str());

        return true;
      }

      class Frame {

      };

      class Transformation{

      public:

        char name[stringSize];
        char from[stringSize];
        char to[stringSize];

        Eigen<double,4,4> transformation;

      private:

      };
    };


  }
}
