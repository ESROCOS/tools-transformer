#ifndef _TRANSFORMER_H_
#define _TRANSFORMER_H_

#include<eigen3/Eigen/Dense>
#include <iostream>

namespace esrocos {
  namespace transformer{

    static Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();
    static Eigen::Matrix4d zeros = Eigen::Matrix4d::Zero();

    template<unsigned int numberOfFrames = 20, unsigned int stringSize = 20>
    class AcyclicTransformer{
    public:

      class Frame;

      class Transformation {

      public:

        Transformation(Frame from, Frame to, const char * id){
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

        Transformation():atob_(identity),btoa_(identity),id_(""),a_(""),b_(""){ }

        const char * id() const {return id_;}
        const char * a() const {return a_;}
        const char * b() const {return b_;}

        const Eigen::Matrix4d atob() const {return atob_;}
        const Eigen::Matrix4d btoa() const {return btoa_;}

        void atob(Eigen::Matrix4d atob){atob_ = atob; btoa_ = atob.inverse();}
        void btoa(Eigen::Matrix4d btoa){btoa_ = btoa; atob_ = btoa.inverse();}

      private:

        Eigen::Matrix4d atob_;
        Eigen::Matrix4d btoa_;

        // +1 for null terminator
        char id_[stringSize+1];
        char a_[stringSize+1];
        char b_[stringSize+1];
      };

      class Frame {

      public:
        Frame(const char * id):id_(""){
          this->id(id);
        }

        const char * id() const {
          return id_;
        }

        bool id(const char * id){
          if (std::strlen(id) > stringSize){
            return false;
          } else {
            std::strcpy(id_,id);
            return true;
          }
        }

        // +1 for null terminator
        char id_[stringSize+1];

        Transformation transformToParent;

        Frame():id_(""){}
      };

      int frames() const {return maxFrames_;}

      int ssize() const {return maxStringSize_;}
      
      AcyclicTransformer(Frame& root){
        currentFrames_ = 1;
        currentTransformations_ = 0;

        if(0 >= maxFrames_) return;
        
	Transformation t(root,root,"root");
        t.atob(identity);
        t.btoa(identity);

        root.transformToParent = t;
        frames_[0] = root;
        return;
      }

      AcyclicTransformer(const char * rootName){
        if(0 >= maxFrames_) return;
        if(std::strlen(rootName) > stringSize) return;

        Frame f(rootName);
        Transformation t(rootName,rootName,"root");
        t.atob(identity);
        t.btoa(identity);

        f.transformToParent = t;
        frames_[0] = f;
        currentFrames_ = 1;
        currentTransformations_ = 0;
        return;
      }

      bool addFrame(Frame f){
        if(currentFrames_ == maxFrames_) return false;
        frames_[currentFrames_] = f;

        addTransformation(&(frames_[currentFrames_].transformToParent));
			
	currentFrames_++;

        return true;
      }

      bool getFrame(const char * id, Frame& f){
        for(int i = 0; i < maxFrames_; i++){
          if (std::strcmp(id,frames_[i].id()) == 0) {
            f = frames_[i];
	    return true;
          }
        }
        return false;
      }

      bool getTransform(const char * frame_a, const char * frame_b, Eigen::Matrix4d& t){
        Frame a,b;

        if(getFrame(frame_a,a) && getFrame(frame_b,b)){
          if(std::strcmp(frame_a,frame_b) == 0) {
            t = identity;
            return true;
          }
          else {

            Eigen::Matrix4d achain[currentFrames_-1];
            Eigen::Matrix4d bchain[currentFrames_-1];

            int acount = 0, bcount = 0;

            for(unsigned int i = 0; i < numberOfFrames-1;i++){
              achain[acount] = a.transformToParent.atob();
              acount++;

	      getFrame(a.transformToParent.b(),a);
            
             
	      if(std::strcmp(a.id(),b.id()) == 0){
		break;
	      } 

              bchain[bcount] = b.transformToParent.btoa();
              bcount++;
              getFrame(b.transformToParent.b(),b);

	      if(std::strcmp(a.id(),b.id()) == 0){ 
                break;
	      } 
            }

            Eigen::Matrix4d result = identity;
 

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

      bool updateTransform(const char * id, Eigen::Matrix4d t){
        for(int i = 0; i < maxFrames_-1; i++){
          if (std::strcmp(id,transforms_[i]->id()) == 0) {
            transforms_[i]->atob(t);
            return true;
          } else {
	  }
	  
        }
        return false;
      }

      bool getTransform(const char * id, Transformation& t){
        for(int i = 0; i < maxTransforms_; i++){
          if (std::strcmp(id,transforms_[i]->id()) == 0) {
            t = *(transforms_[i]);
            return true;
          }
        }
        return false;
      }

    void printAdresses(){
    std::cout << "addresses in transforms:" << std::endl;
      for (int i = 0; i < maxTransforms_; i++){
        std::cout << transforms_[i]->id() <<": " << transforms_[i] << std::endl;
      }

      for (int i = 0; i < maxFrames_; i++){
        std::cout << frames_[i].id() << ": " << &frames_[i] << std::endl;
	std::cout << "\t" << frames_[i].transformToParent.id() << ": " << &(frames_[i].transformToParent) << std::endl;
      }  
    }

    private:

      AcyclicTransformer():currentFrames_(0),currentTransformations_(0){
      }

      bool addTransformation(Transformation * t){

        if(currentTransformations_ == maxTransforms_) return false;
        transforms_[currentTransformations_] = t;
        currentTransformations_++;

        return true;
      }

      int currentFrames_;
      int currentTransformations_;

      static const int maxFrames_ = numberOfFrames;
      static const int maxTransforms_ = numberOfFrames-1;
      static const int maxStringSize_ = stringSize;

      Frame frames_[numberOfFrames];
      Transformation * transforms_[numberOfFrames-1];
    };
  }
}

#endif
