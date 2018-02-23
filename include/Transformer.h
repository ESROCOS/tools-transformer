#ifndef _TRANSFORMER_H_
#define _TRANSFORMER_H_

#include<eigen3/Eigen/Dense>
#include <iostream>

namespace esrocos {
  namespace transformer{

    static Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f zeros = Eigen::Matrix4f::Zero();

    template<unsigned int numberOfFrames = 20, unsigned int stringSize = 20>
    class AcyclicTransformer{
    public:

      class Frame;

      class Transformation{

      public:

        Transformation(Frame from,Frame to, const char * id){
          if (std::strlen(id) > stringSize)
          {
            std::strcpy(id_,"");
            return;
          }

          std::strcpy(a_,from.id_);
          std::strcpy(b_,to.id_);
          std::strcpy(id_,id);

          atob_ = identity;
          btoa_ = identity;
        }

        Transformation(const char * a, const char * b, const char * id){
          if (std::strlen(a) > stringSize || std::strlen(b) > stringSize || std::strlen(id) > stringSize)
          {
            std::strcpy(id_,"");
            return;
          }

          std::strcpy(a_,a);
          std::strcpy(b_,b);
          std::strcpy(id_,id);

          atob_ = identity;
          btoa_ = identity;
        }

        Transformation():id_(""),a_(""),b_(""){
          atob_ = identity;
          btoa_ = identity;
        }

        const char * id(){return id_;}
        const char * a() {return a_;}
        const char * b() {return b_;}

        Eigen::Matrix4f atob(){return atob_;}
        Eigen::Matrix4f btoa(){return btoa_;}

        void atob(Eigen::Matrix4f atob){atob_ = atob; btoa_ = atob.inverse();}
        void btoa(Eigen::Matrix4f btoa){btoa_ = btoa; atob_ = btoa.inverse();}


      private:

        Eigen::Matrix4f atob_;
        Eigen::Matrix4f btoa_;

        // +1 for null terminator
        char id_[stringSize+1];
        char a_[stringSize+1];
        char b_[stringSize+1];
      };

      class Frame {

      public:
        Frame(const char * id):id_(""){
          if (std::strlen(id) > stringSize){
            //do nothing
          } else {
            std::strcpy(id_,id);
          }
        }
        // +1 for null terminator
        char id_[stringSize+1];

        Transformation transformToParent;

        Frame():id_(""){
        }
      };

      int frames() const {return maxFrames_;}

      int ssize() const {return maxStringSize_;}

      AcyclicTransformer(const char * rootName){
        if(currentFrames_ == maxFrames_) return;
        if(std::strlen(rootName) > stringSize) return;

        Frame f(rootName);
        Transformation t(rootName,rootName,"root");
        t.atob_ = identity;
        t.btoa_ = identity;

        f.transformToParent = t;
        frames_[0] = f;
        currentFrames_ = 1;

        return;
      }

      bool addFrame(Frame f){
        if(currentFrames_ == maxFrames_) return false;

        frames_[currentFrames_] = f;
        currentFrames_++;

        addTransformation(f.transformToParent);

        return true;
      }

      bool getFrame(const char * id, Frame & f){
        //std::cout << "get frame with id: " << id << std::endl;
        for(int i = 0; i < maxFrames_; i++){
          //std::cout << "check frame " << i << ": " << frames_[i].id_ << std::endl;
          if (std::strcmp(id,frames_[i].id_) == 0) {
            f = frames_[i];
            return true;
          }
        }
        return false;
      }

      bool getTransform(const char * frame_a, const char * frame_b, Eigen::Matrix4f& t){
        Frame a,b;
        if(getFrame(frame_a,a) && getFrame(frame_b,b)){
          if(std::strcmp(frame_a,frame_b) == 0) {
            t = identity;
            return true;
          }
          else {

            Eigen::Matrix4f achain[currentFrames_-1];
            Eigen::Matrix4f bchain[currentFrames_-1];

            int acount = 0, bcount = 0;

            for(unsigned int i = 0; i < numberOfFrames-1;i++){

              achain[acount] = a.transformToParent.atob_;
              acount++;
              getFrame(a.transformToParent.b_,a);
              if(std::strcmp(a.id_,b.id_) == 0) break;

              bchain[bcount] = b.transformToParent.btoa_;
              bcount++;
              getFrame(b.transformToParent.b_,b);
              if(std::strcmp(a.id_,b.id_) == 0) break;
            }

            Eigen::Matrix4f result = identity;

            for(int i = 0; i < acount; i++){
              result = result * achain[i];
            }

            for(int i = 0; i < bcount; i++){
              result = result * bchain[i];
            }

            t = result;
            return true;
          }
        }
        t = zeros;
        return false;
      }

      bool updateTransform(const char * id, Transformation t){
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

      bool getTransform(const char * id, Transformation& t){
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
