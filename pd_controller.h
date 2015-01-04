//
//  pd_controller.h
//  
//
//  Created by Joshua Lurz on 5/28/14.
//
//

#ifndef _pd_controller_h
#define _pd_controller_h
#include <string>

class PDController {
  public:
    PDController(const std::string& id, const double startTime, const double KD, const double KP):mId(id), mKD(KD), mKP(KP), mPreviousError(0), mPreviousTime(startTime){}
    
    double update(double current, double goal, double currentTime){
        assert(isfinite(current) && isfinite(goal));
        
        // Calculate the error
        double currentError = goal - current;
        assert(isfinite(currentError));
        
        // Calculate the time difference
        double deltaT = currentTime - mPreviousTime;
    
        // Calculate the error derivative
        double dError = 0;
        double ddError = 0;
        
        // Generally initial conditions only.
        if(deltaT > std::numeric_limits<double>::min()){
            dError = (currentError - mPreviousError) / deltaT;
            ddError = (dError - mPreviousDError) / deltaT;
        }
        
        // Calculate the output. This is the difference value, not the absolute
        // value.
        double output = mKP * currentError + mKD * dError;
        assert(isfinite(output));
        
        // Save values for next iteration
        mPreviousTime = currentTime;
        mPreviousError = currentError;
        mPreviousDError = dError;
        return output;
    }
    
    const std::string& getId() const {
        return mId;
    }
    
    double getKD() const {
        return mKD;
    }
    
    double getKP() const {
        return mKP;
    }
    
    void reset(const double currentTime){
        mPreviousTime = currentTime;
        mPreviousError = 0;
        mPreviousDError = 0;
    }
    
    private:
        std::string mId;
        double mKD;
        double mKP;
    
        double mPreviousError;
        double mPreviousDError;
        double mPreviousTime;
};

#endif
