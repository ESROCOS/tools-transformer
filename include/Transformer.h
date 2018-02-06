#ifndef _TRANSFORMER_H_
#define _TRANSFORMER_H_

#include<eigen3/Eigen/Dense>

namespace esrocos {
  namespace transformer{

    template <unsigned int numberOfFrames = 20, unsigned int stringSize = 20> class AcyclicTransformer;

    template<unsigned int numberOfFrames, unsigned int stringSize>
    class AcyclicTransformer{

    public:
    class Frame;
    class Transformation;


    public:

      class Transformation{

        friend class Frame;
        friend class AcyclicTransformer<numberOfFrames, stringSize>;

      public:
        // +1 for null terminator
        char id_[stringSize+1];
        char a_[stringSize+1];
        char b_[stringSize+1];

        Eigen::Matrix<double,4,4> atob;
        Eigen::Matrix<double,4,4> btoa;

        Transformation(const char * a, const char * b, const char * id){
          if (std::strlen(a) > stringSize || std::strlen(b) > stringSize || std::strlen(id) > stringSize)
          {
            std::strcpy(id_,"invalid");
            return;
          }

          std::strcpy(a_,a);
          std::strcpy(b_,b);
          std::strcpy(id_,id);
        }

      private:
        Transformation(){}
      };

      class Frame {
        friend class AcyclicTransformer<numberOfFrames, stringSize>;

      public:
        Frame(const char * id){
          if (std::strlen(id) > stringSize) return;
        }
        // +1 for null terminator
        char id_[stringSize+1];
        Transformation transformToParent;
      private:
        Frame(){}
      };


      int frames() const {return maxFrames_;}
      int ssize() const {return maxStringSize_;}

      AcyclicTransformer(Frame root){
        if(currentFrames_ == maxFrames_) return;

        frames_[currentFrames_] = root;
        currentFrames_++;

        return;
      }

      bool addFrame(Frame f){
        if(currentFrames_ == maxFrames_) return false;

        frames_[currentFrames_] = f;
        currentFrames_++;

        addTransformation(f.transformToParent);

        return true;
      }

      bool getFrame(char * id, Frame & f){
        for(int i = 0; i < maxFrames_; i++){
          if (std::strcmp(id,frames_[i].id) == 0) {
            f = frames_[i];
            return true;
          }
        }
        return false;
      }

      bool getTransform(char * frame_a, char * frame_b, Transformation& t){
        //TODO
        return false;
      }

      bool updateTransform(char * id, Transformation t){
        for(int i = 0; i < maxFrames_-1; i++){
          if (std::strcmp(id,transforms_[i].id) == 0) {
            t = transforms_[i];
            return true;
          }
        }
        return false;
      }

    private:

      AcyclicTransformer(){}

      bool addTransformation(Transformation t){
        if(currentTransformations_ == maxTransforms_) return false;

        transforms_[currentTransformations_] = t;
        currentTransformations_++;

        return true;
      }

      bool getTransform(char * id, Transformation& t){
        for(int i = 0; i < maxTransforms_; i++){
          if (std::strcmp(id,transforms_[i].id) == 0) {
            t = transforms_[i];
            return true;
          }
        }
        return false;
      }

      int currentFrames_;
      int currentTransformations_;

      static const int maxFrames_ = numberOfFrames;
      static const int maxTransforms_ = numberOfFrames-1;
      static const int maxStringSize_ = stringSize;

      Frame frames_[numberOfFrames];
      Transformation transforms_[numberOfFrames-1];
    };
  }
}

#endif
